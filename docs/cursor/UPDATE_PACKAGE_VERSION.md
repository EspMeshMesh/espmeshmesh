# Procedure to update and publish new PlatformIO package version

## Procedure

1. Change `library.json` in two places: the top-level `version` field and the `build.flags` value that sets `ESPMESHMESH_VERSION`. Use the numeric version only (no leading `v`), e.g. `1.2.3`.
2. Commit the changes with message `bump to 1.2.3` where `1.2.3` is the new version.
3. Add an annotated Git tag `v1.2.3` with the same digits. Use the tag name as the tag message (e.g. `-m "v1.2.3"`).
4. Push the branch and tags to `origin`.
5. Publish to the PlatformIO registry using the command below (from the repository root).

## Useful commands

```bash
git tag -a v1.2.3 -m "v1.2.3"
git push origin main
git push origin v1.2.3
~/.platformio/penv/bin/pio package publish --no-interactive
```

Replace `1.2.3` / `v1.2.3` and `main` with the version and branch you are releasing.
