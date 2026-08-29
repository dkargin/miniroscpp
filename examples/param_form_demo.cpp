//
// ParameterForm embedding example.
// A custom page handler owns most of the HTML and appends a ParameterForm fragment.
//

#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "miniros/console.h"
#include "miniros/errors.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/utility/parameter_collection.h"
#include "miniros/http/parameter_form.h"
#include "miniros/io/poll_manager.h"

using namespace miniros;
using namespace miniros::http;

namespace {

struct CameraSettings {
  int fps = 30;
  double time_offset = 0.0;
  std::string binning = "disabled";
  std::string resolution = "1920x1080";
};

CameraSettings g_settings;

/// Page that builds its own HTML and embeds a ParameterForm at the end.
class CameraDashboardHandler : public EndpointHandler {
public:
  explicit CameraDashboardHandler(std::shared_ptr<ParameterForm> form)
    : form_(std::move(form))
  {}

  Error handle(const network::ClientInfo& /*clientInfo*/,
               std::shared_ptr<HttpRequest> request) override
  {
    std::ostringstream ss;
    ss << "<!doctype html>\n<html><head><meta charset=\"utf-8\"/>"
          "<title>Camera node</title>\n"
          "<style>\n"
          "body{font-family:sans-serif;margin:1.5rem;max-width:42rem;}\n"
          "table{border-collapse:collapse;}\n"
          "th,td{text-align:left;padding:0.35rem 0.75rem 0.35rem 0;vertical-align:top;}\n"
          "section.param-form{margin-top:1.5rem;padding-top:1rem;"
          "border-top:1px solid #ccc;}\n"
          ".param-dirty{color:#c60;font-weight:bold;}\n"
          "code{background:#f4f4f4;padding:0.1rem 0.3rem;}\n"
          "</style></head><body>\n";

    ss << "<h1>Camera node dashboard</h1>\n";
    ss << "<p>This page is rendered by a custom <code>EndpointHandler</code>. "
          "Changed fields show <span class=\"param-dirty\">*</span> before Apply. "
          "Try <code>4x4</code> binning with <code>90</code> FPS to see a rejection.</p>\n";
    ss << "<h2>Live status</h2>\n<ul>\n";
    ss << "<li>Applied FPS: <code>" << g_settings.fps << "</code></li>\n";
    ss << "<li>Time offset: <code>" << g_settings.time_offset << "</code> s</li>\n";
    ss << "<li>Binning: <code>" << g_settings.binning << "</code></li>\n";
    ss << "<li>Resolution: <code>" << g_settings.resolution << "</code></li>\n";
    ss << "</ul>\n";

    if (form_)
      form_->renderHtml(ss);

    ss << "</body></html>\n";

    request->setResponseBody(ss.str(), "text/html");
    request->setResponseStatusOk();
    return Error::Ok;
  }

private:
  std::shared_ptr<ParameterForm> form_;
};

} // namespace

int main(int /*argc*/, char** /*argv*/)
{
  console::initializeSafe();

  PollManagerPtr pm = PollManager::instance();
  pm->start();

  HttpServer server(&pm->getPollSet());

  auto params = std::make_shared<ParameterCollection>();
  params->addEnum("fps", {{"30", "30 FPS"}, {"60", "60 FPS"}, {"90", "90 FPS"}}, "30")
      .label("FPS")
      .description("Capture frame rate");
  params->addDouble("time_offset", 0.0)
      .label("Time offset")
      .description("Timestamp offset in seconds");
  params->addEnum("binning",
                  {{"disabled", "No binning"}, {"2x2", "2×2"}, {"4x4", "4×4"}},
                  "disabled")
      .label("Binning")
      .description("Sensor binning mode");
  params->addEnum("resolution",
                  {{"1920x1080", "Full HD"}, {"1280x720", "HD"}, {"640x480", "VGA"}},
                  "1920x1080")
      .label("Resolution")
      .description("Output resolution");

  auto form = std::make_shared<ParameterForm>("camera", "Grabber settings", params);

  form->setApplyCallback([](const ParameterCollection& proposed) {
    ApplyReport report;

    if (proposed.getString("binning") == "4x4" && proposed.getInt("fps") == 90) {
      report.reject("Failed to initialize camera");
      report.rejectField("binning", "4×4 binning is incompatible with 90 FPS");
      report.rejectField("fps", "Reduce FPS when using 4×4 binning");
      return report;
    }

    g_settings.fps = proposed.getInt("fps");
    g_settings.time_offset = proposed.getDouble("time_offset");
    g_settings.binning = proposed.getString("binning");
    g_settings.resolution = proposed.getString("resolution");

    std::cout << "Applied: fps=" << g_settings.fps
              << " time_offset=" << g_settings.time_offset
              << " binning=" << g_settings.binning
              << " resolution=" << g_settings.resolution << std::endl;
    return report;
  });

  constexpr const char* kPagePath = "/";
  constexpr const char* kApplyPath = "/api2/params/camera";

  form->setApplyPath(kApplyPath);
  form->setRedirectPath(kPagePath);
  form->registerApplyEndpoint(server);

  auto page = std::make_shared<CameraDashboardHandler>(form);
  server.registerEndpoint(
    std::make_unique<SimpleFilter>(HttpMethod::Get, kPagePath),
    page,
    nullptr);

  Error err = server.start(8080);
  if (!err) {
    std::cerr << "Failed to start HTTP server: " << err.toString() << std::endl;
    return 1;
  }

  const int port = server.getPort();
  std::cout << "Parameter form demo on http://localhost:" << port << "/\n"
            << "Edit settings (dirty fields show *). Apply commits; Cancel resets.\n"
            << "4x4 + 90 FPS is rejected with field errors.\n";

  std::atomic<bool> running{true};
  while (running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  server.stop();
  pm->shutdown();
  return 0;
}
