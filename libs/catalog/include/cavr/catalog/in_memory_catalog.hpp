#pragma once

// Reference Catalog implementation backed by ordinary containers. Dependency-free;
// used for tests, and as the fallback when the SQLite backend is disabled. It
// keeps no file, so it is not persistent — it exists to pin down the semantics the
// SQLite backend must match.

#include <cavr/catalog/catalog.hpp>

#include <algorithm>
#include <map>
#include <vector>

namespace cavr::catalog {

class InMemoryCatalog final : public Catalog {
 public:
  CatalogStatus initialize() override { return CatalogStatus::success(); }

  CatalogStatus upsert_session(const CatalogSession& session) override {
    if (session.id.empty()) return CatalogStatus::failure("Session id must not be empty");
    sessions_[session.id] = session;
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<CatalogSession> list_sessions() const override {
    std::vector<CatalogSession> out;
    out.reserve(sessions_.size());
    for (const auto& [id, session] : sessions_) out.push_back(session);
    return out;
  }

  [[nodiscard]] std::optional<CatalogSession> find_session(const SessionId& id) const override {
    const auto it = sessions_.find(id);
    if (it == sessions_.end()) return std::nullopt;
    return it->second;
  }

  CatalogStatus add_tag(const SessionId& id, const std::string& tag) override {
    if (sessions_.find(id) == sessions_.end())
      return CatalogStatus::failure("Unknown session: " + id);
    auto& tags = tags_[id];
    if (std::find(tags.begin(), tags.end(), tag) == tags.end()) tags.push_back(tag);
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<std::string> tags_for(const SessionId& id) const override {
    const auto it = tags_.find(id);
    return it == tags_.end() ? std::vector<std::string>{} : it->second;
  }

  CatalogStatus add_annotation(const SessionId& id, const CatalogAnnotation& annotation) override {
    if (sessions_.find(id) == sessions_.end())
      return CatalogStatus::failure("Unknown session: " + id);
    annotations_[id].push_back(annotation);
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<CatalogAnnotation> annotations_for(const SessionId& id) const override {
    const auto it = annotations_.find(id);
    return it == annotations_.end() ? std::vector<CatalogAnnotation>{} : it->second;
  }

  CatalogStatus add_bookmark(const SessionId& id, const CatalogBookmark& bookmark) override {
    if (sessions_.find(id) == sessions_.end())
      return CatalogStatus::failure("Unknown session: " + id);
    bookmarks_[id].push_back(bookmark);
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<CatalogBookmark> bookmarks_for(const SessionId& id) const override {
    const auto it = bookmarks_.find(id);
    return it == bookmarks_.end() ? std::vector<CatalogBookmark>{} : it->second;
  }

  CatalogStatus add_validation_summary(const SessionId& id,
                                       const ValidationSummary& summary) override {
    if (sessions_.find(id) == sessions_.end())
      return CatalogStatus::failure("Unknown session: " + id);
    validations_[id].push_back(summary);
    return CatalogStatus::success();
  }

  [[nodiscard]] std::vector<ValidationSummary> validation_summaries_for(
      const SessionId& id) const override {
    const auto it = validations_.find(id);
    return it == validations_.end() ? std::vector<ValidationSummary>{} : it->second;
  }

  [[nodiscard]] int schema_version() const override { return kCatalogSchemaVersion; }

 private:
  std::map<SessionId, CatalogSession> sessions_;  // ordered for stable listing
  std::map<SessionId, std::vector<std::string>> tags_;
  std::map<SessionId, std::vector<CatalogAnnotation>> annotations_;
  std::map<SessionId, std::vector<CatalogBookmark>> bookmarks_;
  std::map<SessionId, std::vector<ValidationSummary>> validations_;
};

}  // namespace cavr::catalog
