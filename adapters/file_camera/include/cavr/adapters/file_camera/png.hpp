#pragma once

// Minimal, dependency-free PNG decoder — so FileCameraAdapter can replay real
// `.png` frames from disk without pulling in libpng/zlib/OpenCV, keeping the whole
// tree dependency-free. It is a from-scratch reader: a complete DEFLATE inflater
// (stored / fixed-Huffman / dynamic-Huffman blocks, RFC 1951), PNG chunk parsing
// (IHDR/PLTE/IDAT/IEND), per-scanline unfiltering (None/Sub/Up/Average/Paeth), and
// conversion into the project's flat "mono8"/"rgb8"/"rgba8" byte layout.
//
// Scope: 8-bit grayscale (color type 0 → mono8), 8-bit truecolor (2 → rgb8),
// indexed/palette (3 → expanded to rgb8, index bit depths 1/2/4/8), grayscale+alpha
// (4 → rgba8) and truecolor+alpha (6 → rgba8). 16-bit channels and interlacing are
// rejected with an error rather than mis-decoded. CRCs are not verified (decoding
// does not need them). A tiny stored-block `write_png` mirrors write_pgm/write_ppm.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace cavr::adapters::file_camera {

struct PngImage final {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;  // "mono8", "rgb8" or "rgba8"
  std::vector<std::uint8_t> pixels;
};

struct PngLoad final {
  PngImage image;
  bool ok{false};
  std::string error;
};

namespace detail {

// -------------------------------------------------------------- DEFLATE

inline constexpr int kMaxBits = 15;

// Canonical Huffman table built from a list of code lengths (puff-style decode).
struct Huffman final {
  std::array<short, kMaxBits + 1> count{};  // codes of each length
  std::vector<short> symbol;                // symbols sorted by code
};

inline void build_huffman(Huffman& h, const std::vector<int>& lengths) {
  h.count.fill(0);
  for (int len : lengths) h.count[len]++;
  h.count[0] = 0;
  std::array<int, kMaxBits + 1> offsets{};
  offsets[1] = 0;
  for (int len = 1; len < kMaxBits; ++len) offsets[len + 1] = offsets[len] + h.count[len];
  h.symbol.assign(lengths.size(), 0);
  for (int sym = 0; sym < static_cast<int>(lengths.size()); ++sym) {
    if (lengths[static_cast<std::size_t>(sym)] != 0)
      h.symbol[static_cast<std::size_t>(offsets[lengths[static_cast<std::size_t>(sym)]]++)] =
          static_cast<short>(sym);
  }
}

// LSB-first bit reader over the raw DEFLATE stream, with inflate driven off it.
class Inflater final {
 public:
  Inflater(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {}

  // Inflates the whole stream into `out`. Returns false on any malformed input.
  [[nodiscard]] bool inflate(std::vector<std::uint8_t>& out) {
    int final_block = 0;
    do {
      final_block = get_bits(1);
      const int type = get_bits(2);
      if (error_) return false;
      if (type == 0) {
        if (!stored_block(out)) return false;
      } else if (type == 1) {
        if (!compressed_block(out, fixed_lit(), fixed_dist())) return false;
      } else if (type == 2) {
        Huffman lit, dist;
        if (!dynamic_tables(lit, dist)) return false;
        if (!compressed_block(out, lit, dist)) return false;
      } else {
        return false;  // reserved block type
      }
      if (error_) return false;
    } while (!final_block);
    return true;
  }

 private:
  int get_bits(int need) {
    long value = bit_buffer_;
    while (bit_count_ < need) {
      if (byte_pos_ >= size_) { error_ = true; return 0; }
      value |= static_cast<long>(data_[byte_pos_++]) << bit_count_;
      bit_count_ += 8;
    }
    bit_buffer_ = static_cast<int>(value >> need);
    bit_count_ -= need;
    return static_cast<int>(value & ((1L << need) - 1));
  }

  int decode(const Huffman& h) {
    int code = 0, first = 0, index = 0;
    for (int len = 1; len <= kMaxBits; ++len) {
      code |= get_bits(1);
      const int count = h.count[static_cast<std::size_t>(len)];
      if (code - count < first) return h.symbol[static_cast<std::size_t>(index + (code - first))];
      index += count;
      first += count;
      first <<= 1;
      code <<= 1;
    }
    error_ = true;
    return -1;
  }

  bool stored_block(std::vector<std::uint8_t>& out) {
    bit_buffer_ = 0;
    bit_count_ = 0;  // align to byte boundary
    if (byte_pos_ + 4 > size_) return false;
    const int len = data_[byte_pos_] | (data_[byte_pos_ + 1] << 8);
    byte_pos_ += 4;  // skip LEN and its complement NLEN
    if (byte_pos_ + static_cast<std::size_t>(len) > size_) return false;
    out.insert(out.end(), data_ + byte_pos_, data_ + byte_pos_ + len);
    byte_pos_ += static_cast<std::size_t>(len);
    return true;
  }

  bool compressed_block(std::vector<std::uint8_t>& out, const Huffman& lit, const Huffman& dist) {
    static const int len_base[29] = {3,  4,  5,  6,   7,   8,   9,   10,  11,  13,
                                     15, 17, 19, 23,  27,  31,  35,  43,  51,  59,
                                     67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const int len_extra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                      2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const int dist_base[30] = {1,    2,    3,    4,    5,    7,     9,     13,
                                      17,   25,   33,   49,   65,   97,    129,   193,
                                      257,  385,  513,  769,  1025, 1537,  2049,  3073,
                                      4097, 6145, 8193, 12289, 16385, 24577};
    static const int dist_extra[30] = {0, 0, 0,  0,  1,  1,  2,  2,  3,  3,  4,  4,  5,  5,  6,
                                       6, 7, 7,  8,  8,  9,  9,  10, 10, 11, 11, 12, 12, 13, 13};
    for (;;) {
      int sym = decode(lit);
      if (error_) return false;
      if (sym == 256) break;                 // end of block
      if (sym < 256) {
        out.push_back(static_cast<std::uint8_t>(sym));
        continue;
      }
      sym -= 257;
      if (sym >= 29) return false;
      const int length = len_base[sym] + get_bits(len_extra[sym]);
      int dsym = decode(dist);
      if (error_ || dsym >= 30) return false;
      const int distance = dist_base[dsym] + get_bits(dist_extra[dsym]);
      if (static_cast<std::size_t>(distance) > out.size()) return false;
      const std::size_t start = out.size() - static_cast<std::size_t>(distance);
      for (int i = 0; i < length; ++i) out.push_back(out[start + static_cast<std::size_t>(i)]);
    }
    return true;
  }

  bool dynamic_tables(Huffman& lit, Huffman& dist) {
    static const int order[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
    const int hlit = get_bits(5) + 257;
    const int hdist = get_bits(5) + 1;
    const int hclen = get_bits(4) + 4;
    if (error_ || hlit > 286 || hdist > 30) return false;

    std::vector<int> cl_lengths(19, 0);
    for (int i = 0; i < hclen; ++i) cl_lengths[static_cast<std::size_t>(order[i])] = get_bits(3);
    Huffman cl;
    build_huffman(cl, cl_lengths);

    std::vector<int> lengths;
    lengths.reserve(static_cast<std::size_t>(hlit + hdist));
    while (static_cast<int>(lengths.size()) < hlit + hdist) {
      const int sym = decode(cl);
      if (error_) return false;
      if (sym < 16) {
        lengths.push_back(sym);
      } else if (sym == 16) {
        if (lengths.empty()) return false;
        const int prev = lengths.back();
        for (int r = 3 + get_bits(2); r > 0; --r) lengths.push_back(prev);
      } else if (sym == 17) {
        for (int r = 3 + get_bits(3); r > 0; --r) lengths.push_back(0);
      } else {  // 18
        for (int r = 11 + get_bits(7); r > 0; --r) lengths.push_back(0);
      }
    }
    if (static_cast<int>(lengths.size()) != hlit + hdist) return false;

    build_huffman(lit, std::vector<int>(lengths.begin(), lengths.begin() + hlit));
    build_huffman(dist, std::vector<int>(lengths.begin() + hlit, lengths.end()));
    return true;
  }

  static const Huffman& fixed_lit() {
    static const Huffman h = [] {
      std::vector<int> lengths(288);
      for (int i = 0; i < 144; ++i) lengths[static_cast<std::size_t>(i)] = 8;
      for (int i = 144; i < 256; ++i) lengths[static_cast<std::size_t>(i)] = 9;
      for (int i = 256; i < 280; ++i) lengths[static_cast<std::size_t>(i)] = 7;
      for (int i = 280; i < 288; ++i) lengths[static_cast<std::size_t>(i)] = 8;
      Huffman out;
      build_huffman(out, lengths);
      return out;
    }();
    return h;
  }
  static const Huffman& fixed_dist() {
    static const Huffman h = [] {
      Huffman out;
      build_huffman(out, std::vector<int>(30, 5));
      return out;
    }();
    return h;
  }

  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t byte_pos_{0};
  int bit_buffer_{0};
  int bit_count_{0};
  bool error_{false};
};

// -------------------------------------------------------------- PNG glue

[[nodiscard]] inline std::uint32_t be32(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

[[nodiscard]] inline int paeth(int a, int b, int c) {
  const int p = a + b - c;
  const int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
  if (pa <= pb && pa <= pc) return a;
  return pb <= pc ? b : c;
}

}  // namespace detail

[[nodiscard]] inline PngLoad read_png(const std::filesystem::path& path) {
  using namespace detail;
  PngLoad result;

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    result.error = "Failed to open image: " + path.string();
    return result;
  }
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  static const std::uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  if (bytes.size() < 8 || !std::equal(sig, sig + 8, bytes.begin())) {
    result.error = "Not a PNG file: " + path.string();
    return result;
  }

  std::uint32_t width = 0, height = 0;
  int bit_depth = 0, color_type = 0, interlace = 0;
  std::vector<std::uint8_t> palette;   // rgb triples
  std::vector<std::uint8_t> idat;      // concatenated compressed data
  bool have_ihdr = false;

  std::size_t pos = 8;
  while (pos + 8 <= bytes.size()) {
    const std::uint32_t len = be32(&bytes[pos]);
    const std::uint8_t* type = &bytes[pos + 4];
    const std::size_t data_pos = pos + 8;
    if (data_pos + len + 4 > bytes.size()) { result.error = "Truncated PNG chunk"; return result; }
    const std::uint8_t* data = &bytes[data_pos];

    if (std::equal(type, type + 4, "IHDR")) {
      if (len < 13) { result.error = "Bad IHDR"; return result; }
      width = be32(data);
      height = be32(data + 4);
      bit_depth = data[8];
      color_type = data[9];
      interlace = data[12];
      have_ihdr = true;
    } else if (std::equal(type, type + 4, "PLTE")) {
      palette.assign(data, data + len);
    } else if (std::equal(type, type + 4, "IDAT")) {
      idat.insert(idat.end(), data, data + len);
    } else if (std::equal(type, type + 4, "IEND")) {
      break;
    }
    pos = data_pos + len + 4;  // skip data + CRC
  }

  if (!have_ihdr || width == 0 || height == 0) { result.error = "Missing/empty IHDR"; return result; }
  if (interlace != 0) { result.error = "Interlaced PNG not supported: " + path.string(); return result; }

  int channels = 0;
  bool paletted = false;
  switch (color_type) {
    case 0: channels = 1; break;               // grayscale
    case 2: channels = 3; break;               // truecolor
    case 3: channels = 1; paletted = true; break;  // indexed
    case 4: channels = 2; break;               // grayscale + alpha
    case 6: channels = 4; break;               // truecolor + alpha
    default: result.error = "Unsupported PNG color type"; return result;
  }
  const bool sub_byte = paletted && (bit_depth == 1 || bit_depth == 2 || bit_depth == 4);
  if (!(bit_depth == 8 || sub_byte)) {
    result.error = "Only 8-bit channels (or sub-byte palette indices) supported: " + path.string();
    return result;
  }

  // Inflate the zlib stream: 2-byte header, then the raw DEFLATE blocks.
  if (idat.size() < 3) { result.error = "Empty IDAT"; return result; }
  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(width) * height * static_cast<std::size_t>(channels) + height);
  Inflater inflater(idat.data() + 2, idat.size() - 2);
  if (!inflater.inflate(raw)) { result.error = "Corrupt PNG (inflate failed): " + path.string(); return result; }

  const std::size_t bits_per_pixel = static_cast<std::size_t>(channels) * static_cast<std::size_t>(bit_depth);
  const std::size_t stride = (static_cast<std::size_t>(width) * bits_per_pixel + 7) / 8;
  const std::size_t bpp = (bits_per_pixel + 7) / 8;  // filter distance, at least 1 byte
  if (raw.size() < (stride + 1) * height) { result.error = "Truncated PNG pixel data"; return result; }

  // Unfilter in place into a contiguous [height * stride] buffer.
  std::vector<std::uint8_t> unfiltered(static_cast<std::size_t>(height) * stride, 0);
  std::size_t src = 0;
  for (std::uint32_t y = 0; y < height; ++y) {
    const int filter = raw[src++];
    std::uint8_t* row = &unfiltered[static_cast<std::size_t>(y) * stride];
    const std::uint8_t* prev = y == 0 ? nullptr : &unfiltered[(static_cast<std::size_t>(y) - 1) * stride];
    for (std::size_t i = 0; i < stride; ++i) {
      const int x = raw[src++];
      const int a = i >= bpp ? row[i - bpp] : 0;
      const int b = prev ? prev[i] : 0;
      const int c = (prev && i >= bpp) ? prev[i - bpp] : 0;
      int value = 0;
      switch (filter) {
        case 0: value = x; break;
        case 1: value = x + a; break;
        case 2: value = x + b; break;
        case 3: value = x + (a + b) / 2; break;
        case 4: value = x + detail::paeth(a, b, c); break;
        default: result.error = "Unknown PNG filter"; return result;
      }
      row[i] = static_cast<std::uint8_t>(value & 0xFF);
    }
  }

  // Convert to the flat output encoding.
  PngImage& img = result.image;
  img.width = width;
  img.height = height;
  const std::size_t pixel_count = static_cast<std::size_t>(width) * height;

  if (paletted) {
    img.encoding = "rgb8";
    img.pixels.resize(pixel_count * 3);
    for (std::uint32_t y = 0; y < height; ++y) {
      const std::uint8_t* row = &unfiltered[static_cast<std::size_t>(y) * stride];
      for (std::uint32_t x = 0; x < width; ++x) {
        std::size_t index = 0;
        if (bit_depth == 8) {
          index = row[x];
        } else {
          const int per_byte = 8 / bit_depth;
          const std::uint8_t byte = row[x / per_byte];
          const int shift = (per_byte - 1 - static_cast<int>(x % per_byte)) * bit_depth;
          index = (byte >> shift) & ((1 << bit_depth) - 1);
        }
        const std::size_t o = (static_cast<std::size_t>(y) * width + x) * 3;
        if (index * 3 + 2 < palette.size()) {
          img.pixels[o] = palette[index * 3];
          img.pixels[o + 1] = palette[index * 3 + 1];
          img.pixels[o + 2] = palette[index * 3 + 2];
        }
      }
    }
  } else if (color_type == 0) {
    img.encoding = "mono8";
    img.pixels.assign(unfiltered.begin(), unfiltered.end());  // stride == width here
  } else if (color_type == 2) {
    img.encoding = "rgb8";
    img.pixels.assign(unfiltered.begin(), unfiltered.end());  // stride == width*3
  } else if (color_type == 4) {  // gray + alpha -> rgba8
    img.encoding = "rgba8";
    img.pixels.resize(pixel_count * 4);
    for (std::size_t i = 0; i < pixel_count; ++i) {
      const std::uint8_t g = unfiltered[i * 2], al = unfiltered[i * 2 + 1];
      img.pixels[i * 4] = g;
      img.pixels[i * 4 + 1] = g;
      img.pixels[i * 4 + 2] = g;
      img.pixels[i * 4 + 3] = al;
    }
  } else {  // color_type == 6, rgba
    img.encoding = "rgba8";
    img.pixels.assign(unfiltered.begin(), unfiltered.end());  // stride == width*4
  }

  result.ok = true;
  return result;
}

// A tiny PNG writer that stores pixels in an uncompressed DEFLATE block (no
// Huffman compression needed), mirroring write_pgm/write_ppm and handy for tests.
// `channels` is 1 (mono8) or 3 (rgb8).
[[nodiscard]] inline bool write_png(const std::filesystem::path& path, std::uint32_t width,
                                    std::uint32_t height, int channels,
                                    const std::vector<std::uint8_t>& pixels) {
  if (channels != 1 && channels != 3) return false;
  if (pixels.size() != static_cast<std::size_t>(width) * height * static_cast<std::size_t>(channels)) return false;

  auto crc32 = [](const std::uint8_t* p, std::size_t n) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
      crc ^= p[i];
      for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
    }
    return crc ^ 0xFFFFFFFFu;
  };
  auto put32 = [](std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(static_cast<std::uint8_t>(x >> 24));
    v.push_back(static_cast<std::uint8_t>(x >> 16));
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x));
  };
  auto chunk = [&](std::vector<std::uint8_t>& out, const char* type, const std::vector<std::uint8_t>& data) {
    put32(out, static_cast<std::uint32_t>(data.size()));
    std::vector<std::uint8_t> typed(type, type + 4);
    typed.insert(typed.end(), data.begin(), data.end());
    out.insert(out.end(), typed.begin(), typed.end());
    put32(out, crc32(typed.data(), typed.size()));
  };

  // Filtered raw scanlines: filter byte 0 (None) + row bytes.
  std::vector<std::uint8_t> raw;
  const std::size_t rowbytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(channels);
  for (std::uint32_t y = 0; y < height; ++y) {
    raw.push_back(0);
    raw.insert(raw.end(), pixels.begin() + static_cast<std::ptrdiff_t>(y * rowbytes),
               pixels.begin() + static_cast<std::ptrdiff_t>((y + 1) * rowbytes));
  }

  // zlib wrapper around one or more stored DEFLATE blocks + Adler-32.
  std::vector<std::uint8_t> zlib;
  zlib.push_back(0x78);
  zlib.push_back(0x01);
  std::size_t off = 0;
  while (off < raw.size() || raw.empty()) {
    const std::size_t block = std::min<std::size_t>(raw.size() - off, 0xFFFF);
    const bool last = off + block >= raw.size();
    zlib.push_back(last ? 1 : 0);  // BFINAL, BTYPE=00 (stored)
    zlib.push_back(static_cast<std::uint8_t>(block & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>(block >> 8));
    zlib.push_back(static_cast<std::uint8_t>(~block & 0xFF));
    zlib.push_back(static_cast<std::uint8_t>((~block >> 8) & 0xFF));
    zlib.insert(zlib.end(), raw.begin() + static_cast<std::ptrdiff_t>(off),
                raw.begin() + static_cast<std::ptrdiff_t>(off + block));
    off += block;
    if (last) break;
  }
  std::uint32_t a = 1, b = 0;
  for (std::uint8_t byte : raw) { a = (a + byte) % 65521; b = (b + a) % 65521; }
  const std::uint32_t adler = (b << 16) | a;
  zlib.push_back(static_cast<std::uint8_t>(adler >> 24));
  zlib.push_back(static_cast<std::uint8_t>(adler >> 16));
  zlib.push_back(static_cast<std::uint8_t>(adler >> 8));
  zlib.push_back(static_cast<std::uint8_t>(adler));

  std::vector<std::uint8_t> ihdr;
  put32(ihdr, width);
  put32(ihdr, height);
  ihdr.push_back(8);                                 // bit depth
  ihdr.push_back(channels == 1 ? 0 : 2);             // color type
  ihdr.push_back(0);                                 // compression
  ihdr.push_back(0);                                 // filter
  ihdr.push_back(0);                                 // interlace

  std::vector<std::uint8_t> out = {137, 80, 78, 71, 13, 10, 26, 10};
  chunk(out, "IHDR", ihdr);
  chunk(out, "IDAT", zlib);
  chunk(out, "IEND", {});

  std::ofstream os(path, std::ios::binary);
  if (!os) return false;
  os.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return static_cast<bool>(os);
}

}  // namespace cavr::adapters::file_camera
