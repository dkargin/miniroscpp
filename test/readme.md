# MiniROS Tests #

Running advanced tests from build folder: `ctest --test-dir test/roscpp/advanced -V --timeout 100`

## Multimaster integration

`test/multimaster/` launches two `miniroscore` instances and two helper nodes via `miniros::Launcher`:

- `mm_requester` — publishes a nonce to `/mm/request`, waits for `ack:<nonce>` on `/mm/response`
- `mm_responder` — echoes requests as acknowledgements on `/mm/response`
- `multimaster_integration_test` — verifies cross-master pub/sub (requester on master A, responder on master B)

```bash
cmake --build build --target multimaster_integration_test
./build/bin/multimaster_integration_test
# or
ctest --test-dir build -R multimaster-integration -V
```

Uses ports `11411` / `11511` (not production `11311`).
