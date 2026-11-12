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
- Fix a reported problem or a clear gap. Open an issue first for larger features, so we can
  agree on the approach before you build it.
- Follow the surrounding code. Add a comment only where the code can't explain itself, such
  as a hardware quirk or an external constraint.
- Write commit titles as `type(scope): summary`, for example `fix(runtime): ...`, in at most
  72 characters.
- Describe the change briefly in your own words, with what you tested: commands, device,
  firmware and GPU driver, and before/after numbers for performance changes.

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
