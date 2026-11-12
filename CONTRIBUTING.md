# Contributing

Contributions are welcome: bug reports, fixes, support for more games and performance work.

## Understand what you submit

Be ready to explain every line you add or remove, and why, in your own words. AI assistants
are fine to use: review what they write as you would anyone else's code, and say in the pull
request that you used one. If you can't explain a change yet, open an issue or a draft pull
request and ask; that is a good way to start.

## Pull requests

- Keep each pull request to one change a reviewer can read line by line. Keep related changes
  together and unrelated ones apart.
- Fix a reported problem or a clear gap. For larger features, open an issue first to agree on
  the approach before building it.
- Follow the surrounding code.
- Say in your own words what the change does and how you tested it. For performance changes,
  include before/after numbers and the device.

## Testing

- Run the host tests: `cmake --build build/host && ctest --test-dir build/host`.
- Test runtime, backend and graphics changes on a handheld as well, or say what you couldn't
  test.

## Game data

Keep game files out of the repository: disc images, executables, extracted assets, saves,
and code copied from a game or the console SDK. Per-game configuration and symbol lists live
in the port repositories.

## License

Contributions are licensed under the GNU General Public License v3.0, like the rest of the
project.
