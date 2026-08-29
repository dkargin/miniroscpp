//
// Parameter form: HTML fragment + Apply POST over a ParameterCollection.
//

#ifndef MINIROS_HTTP_PARAMETER_FORM_H
#define MINIROS_HTTP_PARAMETER_FORM_H

#include <functional>
#include <memory>
#include <mutex>
#include <ostream>
#include <string>

#include "miniros/errors.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/utility/parameter_collection.h"
#include "miniros/macros.h"

namespace miniros {

class CallbackQueue;

namespace http {

class HttpServer;
class HttpRequest;

/// HTTP UI for editing a ParameterCollection.
///
/// Embed into a larger page with renderHtml(). Register a POST endpoint with
/// registerApplyEndpoint() so the form can submit independently of the page handler.
class MINIROS_DECL ParameterForm : public std::enable_shared_from_this<ParameterForm> {
public:
  /// Called with proposed values (not yet committed). Return a non-ok ApplyReport to reject.
  using ApplyFn = std::function<ApplyReport(const ParameterCollection& proposed)>;

  /// @param id - stable form id (HTML id prefix).
  /// @param title - heading shown above the form (optional).
  /// @param params - parameter storage; created empty if null.
  explicit ParameterForm(std::string id, std::string title = {},
                         std::shared_ptr<ParameterCollection> params = nullptr);

  const std::string& id() const { return id_; }
  const std::string& title() const { return title_; }
  const std::string& applyPath() const { return apply_path_; }
  const std::string& redirectPath() const { return redirect_path_; }

  std::shared_ptr<ParameterCollection> parameters() const { return params_; }

  void setApplyCallback(ApplyFn cb);

  /// Where the HTML form POSTs. Required before registerApplyEndpoint / renderHtml.
  void setApplyPath(std::string path);
  /// Where to redirect after Apply (success or rejection). Usually the embedding page.
  void setRedirectPath(std::string path);

  /// Register POST handler for apply_path_. Must be called on a shared_ptr-owned instance.
  void registerApplyEndpoint(HttpServer& server,
                             const std::shared_ptr<CallbackQueue>& cb = nullptr);

  /// Append an HTML fragment (section + form). Not a full document.
  void renderHtml(std::ostream& out) const;

  /// Parse form body, validate, invoke ApplyFn; commit only on success.
  ApplyReport applyFromRequest(const std::shared_ptr<HttpRequest>& request);

  /// Last apply outcome (cleared after a successful apply, or manually).
  ApplyReport lastReport() const;
  void clearLastReport();

private:
  friend class ParameterFormApplyHandler;

  void renderDirtyScript(std::ostream& out, const std::string& formDomId) const;

  std::string id_;
  std::string title_;
  std::string apply_path_;
  std::string redirect_path_;
  ApplyFn apply_cb_;
  std::shared_ptr<ParameterCollection> params_;

  mutable std::mutex mutex_;
  ApplyReport last_report_;
  /// Values shown after a rejected apply (keeps user edits); empty when none.
  std::shared_ptr<ParameterCollection> draft_;
};

/// POST handler that applies ParameterForm and redirects back to the page.
class MINIROS_DECL ParameterFormApplyHandler : public EndpointHandler {
public:
  explicit ParameterFormApplyHandler(std::shared_ptr<ParameterForm> form);

  Error handle(const network::ClientInfo& clientInfo,
               std::shared_ptr<HttpRequest> request) override;

private:
  std::shared_ptr<ParameterForm> form_;
};

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_PARAMETER_FORM_H
