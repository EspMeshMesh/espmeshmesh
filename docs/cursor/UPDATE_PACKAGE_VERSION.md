# Procedure to update and publish new PlatformIO package version

## Procedure

as message.
1. Change library.json file in two places: near *version* and near *build.flags* with the current version without the initial v
2. Commit the changes with message bump to 1.2.3 where 1.2.3 is the current version.
3. Add a tag formatted as v1.2.3 where 1.2.3 is an example version; use the name of the tag 4. Push changes with tags on the origin repository
5. Use pio commands to push changes on the PlatformIO registry.

## Useful commands

```bash
~/.platformio/penv/bin/pio package publish --no-interactive
```
