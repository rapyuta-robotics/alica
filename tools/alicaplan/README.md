# alicaplan

Compile, check and decompile ALICA plans.

```sh
uv tool install ./tools/alicaplan          # or: uv run --project tools/alicaplan alicaplan ...
alicaplan compile plans/src -o plans/etc   # *.alica.yaml -> engine JSON, then check it
alicaplan check alica_tests/etc            # check any folder of engine JSON
alicaplan decompile alica_tests/etc -o src # engine JSON -> *.alica.yaml
```

The source format, the checks and the layout files are described in
[docs/alicaplan.md](../../docs/alicaplan.md). A complete example lives in
[examples/football](examples/football/football.alica.yaml).

Run the tests with `uv run pytest`. The engine-load tests need `alica_load_plans`, which
`scripts/build.sh` installs into `install/bin`; set `ALICA_LOAD_PLANS` to use another binary.
