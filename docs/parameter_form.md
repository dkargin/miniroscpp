# Parameter collection and HTTP forms

This document describes the embeddable settings UI for miniROS HTTP servers:
typed parameter storage (`ParameterCollection`) and HTML form rendering / Apply
handling (`ParameterForm`). Intended as handoff notes for implementers and agents.

## Purpose

Nodes (camera grabbers, etc.) expose a set of typed settings in a web UI. The
operator edits fields, presses **Apply** to commit (with optional rejection and
field errors), or **Cancel** to discard edits. Multiple forms can be embedded
into a custom page owned by another `EndpointHandler`.

This is **not** the ROS parameter server. Values live in `ParameterCollection`
and are pushed into device logic via an Apply callback.

## Files

| Path | Role |
|------|------|
| `include/miniros/utility/parameter_collection.h` | `ParamSpec`, `ApplyReport`, `ParamSpecRef`, `ParameterCollection` |
| `src/utility/parameter_collection.cpp` | Storage, getters/setters, fluent registration, YAML |
| `include/miniros/utility/yaml.h` | `parseYaml` / `loadYamlFile` → `RpcValue` |
| `include/miniros/http/parameter_form.h` | `ParameterForm`, `ParameterFormApplyHandler` |
| `src/http/parameter_form.cpp` | HTML fragment, POST apply, dirty-marker JS |
| `examples/param_form_demo.cpp` | Dashboard page that embeds a form |
| `examples/examples.md` | How to run the demo |

Built into `libroscxx` (`src/CMakeLists.txt`). Demo: `-DMINIROS_BUILD_EXAMPLES=ON`,
binary `param_form_demo`.

`ParameterCollection` lives in `miniros`. `ParameterForm` stays in `miniros::http`.

## Architecture

```
ParameterCollection     ← source of truth (committed values)
        ▲
        │ shared_ptr
ParameterForm           ← HTML fragment + Apply POST + last ApplyReport / draft
        ▲
        │ renderHtml() append
Custom EndpointHandler  ← owns full HTML page (status, layout, multiple forms)
```

- **Collection** = data model (names, types, values, labels).
- **Form** = HTTP/UI adapter over one collection.
- **Page handler** = composes the document; calls `form->renderHtml(out)` where
  the settings block should appear.

Apply is a **separate POST** endpoint so embedding does not require the page
handler to parse form bodies.

## ParameterCollection

Thread-safe (`std::mutex`). Holds an ordered list of `ParamSpec`.

### ParamSpec fields

| Field | Meaning |
|-------|---------|
| `name` | Stable id: HTML `name=`, getters, `ApplyReport` field keys. **Do not localize.** |
| `label` | UI caption; `displayLabel()` falls back to `name`. Candidate for i18n. |
| `description` | Longer help (HTML `title=` tooltip). Candidate for i18n. |
| `type` | `Bool`, `Int`, `Double`, `Enum` |
| values | `bool_value` / `int_value` / `double_value` / `string_value` (enum code) |
| bounds | `has_min` / `has_max` with `int_min`/`int_max` or `double_min`/`double_max` |
| `enum_options` | `{ code, description }` for enums |

`valueAsString()` is the canonical form for HTML defaults / dirty comparison
(bool → `"1"`/`"0"`).

### Fluent registration (`ParamSpecRef`)

Adders take only name + default (and options for enum). They return `ParamSpecRef`
(collection pointer + index), which chains metadata:

```cpp
auto params = std::make_shared<ParameterCollection>();

params->addBool("enabled", false)
    .label("Enabled")
    .description("Most important bool");

params->addInt("width", 1920)
    .label("Width")
    .min(1)
    .max(4096);

params->addDouble("time_offset", 0.0)
    .label("Time offset")
    .description("Timestamp offset in seconds");

params->addEnum("fps",
                {{"30", "30 FPS"}, {"60", "60 FPS"}, {"90", "90 FPS"}},
                "30")
    .label("FPS")
    .description("Capture frame rate");
```

- `ParamSpecRef` is for **append-only** registration; do not keep it across
  erasures (there is no erase API today).
- Each setter locks the collection.
- Empty `selected` on `addEnum` → first option.

### Reading / writing values

```cpp
int fps = params->getInt("fps");           // Int, or numeric Enum code
double t = params->getDouble("time_offset");
std::string b = params->getString("binning");  // via valueAsString()
params->setString("binning", "2x2");
```

`getInt` on an enum parses `string_value` as decimal (e.g. code `"60"` → `60`).
Missing / wrong type throws `std::runtime_error` on getters; setters return `Error`.

`copyValuesFrom(other)` copies values for matching names/types (used on successful Apply).

`toYaml(fileComment)` returns YAML text. Each parameter is preceded by a `#`
comment from its description (or display label). `loadYaml` / `loadYamlFile`
overlay matching keys via `miniros::parseYaml`.

## ApplyReport

Returned by the Apply callback (and by parse failures):

```cpp
ApplyReport report;
report.reject("Failed to initialize camera");           // generic banner
report.rejectField("fps", "Reduce FPS when using 4×4"); // next to field
return report;  // ok() == false → collection not updated
```

`ok()` is true only when `error == Ok`, `message` empty, and `field_errors` empty.

## ParameterForm

Must be owned by `std::shared_ptr` (`enable_shared_from_this` for apply endpoint).

### Setup

```cpp
auto form = std::make_shared<ParameterForm>("camera", "Grabber settings", params);
form->setApplyPath("/api2/params/camera");
form->setRedirectPath("/");   // usually the embedding page
form->setApplyCallback([](const ParameterCollection& proposed) -> ApplyReport {
  ApplyReport report;
  // validate / reconfigure hardware using proposed.get*()
  // on failure: report.reject(...); report.rejectField(...); return report;
  // on success: copy into device state; return report; // ok
  return report;
});
form->registerApplyEndpoint(server);  // POST only
```

`ApplyFn` receives **proposed** values (not yet committed). Commit happens only
if `report.ok()`.

### Embedding into a custom page

`renderHtml(ostream&)` emits a **fragment** (`<section>…</section>`), not a full
document:

```cpp
class MyPage : public EndpointHandler {
  Error handle(..., std::shared_ptr<HttpRequest> request) override {
    std::ostringstream ss;
    ss << "<!doctype html><html>...<h1>Dashboard</h1>...";
    form_->renderHtml(ss);   // settings block at the end (or anywhere)
    ss << "</body></html>";
    request->setResponseBody(ss.str(), "text/html");
    request->setResponseStatusOk();
    return Error::Ok;
  }
};
```

Multiple forms: call `renderHtml` for each; each needs its own `applyPath`.

### Apply / Cancel UX

| Action | Behavior |
|--------|----------|
| **Apply** | `POST` `application/x-www-form-urlencoded` to `applyPath`. Validate → callback → on success update collection and clear draft/errors; on failure keep **draft** values and `lastReport`. Always **303** redirect to `redirectPath` (with `Content-Length: 0`). |
| **Cancel** | Client-side: restore controls from `data-original` (committed values), clear dirty `*`, hide error divs. No server round-trip. |

After rejection, the next page GET shows draft values, generic error, and
per-field errors. Successful Apply clears draft and errors.

### Dirty marker `*`

Small inline JS (per form) compares each input to `data-original` (committed
`valueAsString()`). Differing fields show `*` next to the label.

HTML escaping uses `miniros::http::xmlEncode` from `http_tools`.

### lastReport / clearLastReport

Page handlers can call `form->lastReport()` if they want to render errors outside
the form fragment. `clearLastReport()` drops report + draft.

## Demo

```bash
cmake -S . -B build -DMINIROS_BUILD_EXAMPLES=ON
cmake --build build --target param_form_demo
./build/bin/param_form_demo
# open http://localhost:8080/
```

Demo page shows live applied status + embedded grabber form. Rejection example:
**4×4 binning + 90 FPS** → generic “Failed to initialize camera” plus field errors.

## Design notes / future

- **i18n**: keep `name` / enum `code` stable; translate `label`, `description`,
  enum option descriptions, Apply/Cancel chrome, and `ApplyReport` text (or switch
  reports to stable codes later). Locale can be chosen per request when building
  the collection or resolving strings at `renderHtml` time.
- **JSON twin**: multimaster-style `GET /api2/params/<id>` schema+values is not
  implemented yet; Apply is form-urlencoded only.
- **ROS param sync**: optional inside `ApplyFn` via `NodeHandle` / `MasterLink`;
  not built into the collection.
- **Template engines** (Jinja2, etc.): same split — collection as model/JSON;
  external template as view. Current path is C++ `renderHtml` fragments to match
  existing master HTML style.

## Quick checklist for a new node

1. Create `shared_ptr<ParameterCollection>`, register params with fluent API.
2. Create `shared_ptr<ParameterForm>(id, title, params)`.
3. `setApplyPath` / `setRedirectPath` / `setApplyCallback` / `registerApplyEndpoint`.
4. In your GET page handler, build HTML and call `form->renderHtml(ss)`.
5. Register the page `EndpointHandler` on `HttpServer` (node: often
   `RPCManager::instance()->getHttpServer()`).
