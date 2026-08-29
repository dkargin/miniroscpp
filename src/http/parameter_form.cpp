//
// ParameterForm implementation.
//

#include "miniros/http/parameter_form.h"

#include "miniros/http/http_filters.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_tools.h"

#include <map>
#include <sstream>

namespace miniros {
namespace http {

ParameterForm::ParameterForm(std::string id, std::string title, std::shared_ptr<ParameterCollection> params)
  : id_(std::move(id))
  , title_(std::move(title))
  , params_(params ? std::move(params) : std::make_shared<ParameterCollection>())
{}

void ParameterForm::setApplyCallback(ApplyFn cb)
{
  std::lock_guard lock(mutex_);
  apply_cb_ = std::move(cb);
}

void ParameterForm::setApplyPath(std::string path)
{
  std::lock_guard lock(mutex_);
  apply_path_ = std::move(path);
}

void ParameterForm::setRedirectPath(std::string path)
{
  std::lock_guard lock(mutex_);
  redirect_path_ = std::move(path);
}

void ParameterForm::registerApplyEndpoint(HttpServer& server, const std::shared_ptr<CallbackQueue>& cb)
{
  std::string path;
  {
    std::lock_guard lock(mutex_);
    path = apply_path_;
  }
  if (path.empty())
    return;

  auto handler = std::make_shared<ParameterFormApplyHandler>(shared_from_this());
  server.registerEndpoint(
    std::make_unique<SimpleFilter>(HttpMethod::Post, path),
    handler,
    cb);
}

ApplyReport ParameterForm::lastReport() const
{
  std::lock_guard lock(mutex_);
  return last_report_;
}

void ParameterForm::clearLastReport()
{
  std::lock_guard lock(mutex_);
  last_report_ = {};
  draft_.reset();
}

void ParameterForm::renderDirtyScript(std::ostream& out, const std::string& formDomId) const
{
  // Marks labels with '*' when the control value differs from data-original (committed).
  // Cancel restores controls to data-original and hides previous apply errors.
  out << "<script>(function(){\n"
         "var root=document.getElementById('" << xmlEncode(formDomId) << "');\n"
         "if(!root)return;\n"
         "var form=root.querySelector('form');\n"
         "if(!form)return;\n"
         "function cur(el){\n"
         "  if(el.type==='checkbox')return el.checked?'1':'0';\n"
         "  return String(el.value);\n"
         "}\n"
         "function sync(){\n"
         "  root.querySelectorAll('[data-param]').forEach(function(row){\n"
         "    var el=row.querySelector('input,select');\n"
         "    var mark=row.querySelector('.param-dirty');\n"
         "    if(!el||!mark)return;\n"
         "    var orig=el.getAttribute('data-original');\n"
         "    mark.textContent=(cur(el)!==orig)?'*':'';\n"
         "  });\n"
         "}\n"
         "form.addEventListener('input',sync);\n"
         "form.addEventListener('change',sync);\n"
         "form.addEventListener('reset',function(ev){\n"
         "  ev.preventDefault();\n"
         "  root.querySelectorAll('[data-param]').forEach(function(row){\n"
         "    var el=row.querySelector('input,select');\n"
         "    if(!el)return;\n"
         "    var orig=el.getAttribute('data-original');\n"
         "    if(el.type==='checkbox')el.checked=(orig==='1');\n"
         "    else el.value=orig;\n"
         "  });\n"
         "  root.querySelectorAll('.param-form-error,.param-field-error')"
         ".forEach(function(n){n.style.display='none';});\n"
         "  sync();\n"
         "});\n"
         "sync();\n"
         "})();</script>\n";
}

void ParameterForm::renderHtml(std::ostream& out) const
{
  ApplyReport report;
  std::shared_ptr<ParameterCollection> view;
  std::string applyPath;
  {
    std::lock_guard lock(mutex_);
    report = last_report_;
    applyPath = apply_path_;
    view = draft_ ? draft_ : params_;
  }
  if (!view)
    return;

  const auto committed = params_->specs();
  std::map<std::string, std::string> original;
  for (const ParamSpec& p : committed)
    original[p.name] = p.valueAsString();

  const std::string formDomId = "param-form-" + id_;
  out << "<section class=\"param-form\" id=\"" << xmlEncode(formDomId) << "\">\n";
  if (!title_.empty())
    out << "  <h2>" << xmlEncode(title_) << "</h2>\n";

  if (!report.ok()) {
    out << "  <div class=\"param-form-error\" style=\"color:#a00;margin:0.5rem 0;\">\n";
    if (!report.message.empty())
      out << "    <p><strong>" << xmlEncode(report.message) << "</strong></p>\n";
    else
      out << "    <p><strong>Apply rejected</strong></p>\n";
    out << "  </div>\n";
  }

  out << "  <form method=\"POST\" action=\"" << xmlEncode(applyPath) << "\">\n";
  out << "    <table>\n";

  for (const ParamSpec& p : view->specs()) {
    const std::string inputId = id_ + "__" + p.name;
    const std::string orig = original.count(p.name) ? original[p.name] : p.valueAsString();
    const auto ferr = report.field_errors.find(p.name);

    out << "      <tr class=\"param-row\" data-param=\"" << xmlEncode(p.name)
        << "\" title=\"" << xmlEncode(p.description) << "\">\n";
    out << "        <th><label for=\"" << xmlEncode(inputId) << "\">"
        << xmlEncode(p.displayLabel())
        << " <span class=\"param-dirty\" aria-hidden=\"true\"></span></label></th>\n";
    out << "        <td>";

    if (p.type == ParamType::Bool) {
      out << "<input type=\"checkbox\" name=\"" << xmlEncode(p.name) << "\" id=\""
          << xmlEncode(inputId) << "\" value=\"1\" data-original=\"" << xmlEncode(orig) << "\"";
      if (p.bool_value)
        out << " checked";
      out << "/>";
    } else if (p.type == ParamType::Int) {
      out << "<input type=\"number\" name=\"" << xmlEncode(p.name) << "\" id=\""
          << xmlEncode(inputId) << "\" value=\"" << p.int_value
          << "\" data-original=\"" << xmlEncode(orig) << "\"";
      if (p.has_min)
        out << " min=\"" << p.int_min << "\"";
      if (p.has_max)
        out << " max=\"" << p.int_max << "\"";
      out << "/>";
    } else if (p.type == ParamType::Double) {
      out << "<input type=\"number\" step=\"any\" name=\"" << xmlEncode(p.name)
          << "\" id=\"" << xmlEncode(inputId) << "\" value=\"" << p.double_value
          << "\" data-original=\"" << xmlEncode(orig) << "\"";
      if (p.has_min)
        out << " min=\"" << p.double_min << "\"";
      if (p.has_max)
        out << " max=\"" << p.double_max << "\"";
      out << "/>";
    } else if (p.type == ParamType::Enum) {
      out << "<select name=\"" << xmlEncode(p.name) << "\" id=\"" << xmlEncode(inputId)
          << "\" data-original=\"" << xmlEncode(orig) << "\">";
      for (const EnumOption& opt : p.enum_options) {
        out << "<option value=\"" << xmlEncode(opt.code) << "\"";
        if (opt.code == p.string_value)
          out << " selected";
        out << ">" << xmlEncode(opt.description.empty() ? opt.code : opt.description)
            << "</option>";
      }
      out << "</select>";
    }

    if (ferr != report.field_errors.end()) {
      out << "<div class=\"param-field-error\" style=\"color:#a00;font-size:0.9em;\">"
          << xmlEncode(ferr->second) << "</div>";
    }

    out << "</td>\n";
    out << "      </tr>\n";
  }

  out << "    </table>\n";
  out << "    <p>\n";
  out << "      <button type=\"submit\">Apply</button>\n";
  out << "      <button type=\"reset\">Cancel</button>\n";
  out << "    </p>\n";
  out << "  </form>\n";
  renderDirtyScript(out, formDomId);
  out << "</section>\n";
}

ApplyReport ParameterForm::applyFromRequest(const std::shared_ptr<HttpRequest>& request)
{
  ApplyReport report;
  if (!request || !params_) {
    report.reject("Internal error", Error::InternalError);
    std::lock_guard lock(mutex_);
    last_report_ = report;
    return report;
  }

  const auto submitted = parseUrlEncoded(request->requestBody());
  ParameterCollection proposed = *params_;

  for (const ParamSpec& spec : params_->specs()) {
    auto it = submitted.find(spec.name);
    if (spec.type == ParamType::Bool) {
      const std::string raw = (it == submitted.end()) ? "0" : it->second;
      if (Error err = proposed.assignFromString(spec.name, raw); !err) {
        report.rejectField(spec.name, "Invalid boolean value", err);
      }
    } else {
      if (it == submitted.end()) {
        report.rejectField(spec.name, "Missing value", Error::InvalidValue);
        continue;
      }
      if (Error err = proposed.assignFromString(spec.name, it->second); !err) {
        report.rejectField(spec.name, "Invalid value", err);
      }
    }
  }

  if (!report.ok()) {
    auto draft = std::make_shared<ParameterCollection>(proposed);
    std::lock_guard lock(mutex_);
    last_report_ = report;
    draft_ = std::move(draft);
    return report;
  }

  ApplyFn cb;
  {
    std::lock_guard lock(mutex_);
    cb = apply_cb_;
  }

  if (cb) {
    try {
      report = cb(proposed);
    } catch (...) {
      report = {};
      report.reject("Apply callback threw", Error::InternalError);
    }
  }

  if (!report.ok()) {
    auto draft = std::make_shared<ParameterCollection>(proposed);
    std::lock_guard lock(mutex_);
    last_report_ = report;
    draft_ = std::move(draft);
    return report;
  }

  params_->copyValuesFrom(proposed);
  {
    std::lock_guard lock(mutex_);
    last_report_ = {};
    draft_.reset();
  }
  return report;
}

ParameterFormApplyHandler::ParameterFormApplyHandler(std::shared_ptr<ParameterForm> form)
  : form_(std::move(form))
{}

Error ParameterFormApplyHandler::handle(const network::ClientInfo& /*clientInfo*/,
                                        std::shared_ptr<HttpRequest> request)
{
  if (!form_ || !request)
    return Error::InternalError;

  if (request->method() != HttpMethod::Post) {
    request->setResponseStatus(405, "Method Not Allowed");
    request->setResponseBody("POST required", "text/plain");
    return Error::Ok;
  }

  // Always redirect back so the embedding page can show success or field errors.
  (void)form_->applyFromRequest(request);

  const std::string redirect = form_->redirectPath().empty() ? "/" : form_->redirectPath();
  request->setResponseStatus(303, "See Other");
  request->setResponseHeader("Location", redirect);
  // Explicit length: empty body would otherwise omit Content-Length and stall keep-alive clients.
  request->setResponseHeader("Content-Length", "0");
  request->setResponseBody("", "text/plain");
  return Error::Ok;
}

} // namespace http
} // namespace miniros
