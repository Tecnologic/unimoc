---
name: file-docs
description: "Use when documenting C++ headers or source files with Doxygen comments, including file, namespace, class, and public API documentation."
---

# file-docs

This skill documents an existing C++ file with focused Doxygen comments. Analyze the code first, then add only information that is useful to someone using or maintaining the API. Preserve the repository's formatting, naming, license, and include conventions.

## Scope

- For a header file, document the file purpose, relevant namespace or group, and public classes, functions, types, and members.
- For a source file, document the file purpose and significant file-scope functions or implementation details. Do not add `#pragma once` to a source file.
- Document private or local variables and functions only when their purpose or invariants are not clear from the code.
- Preserve existing comments and improve them only when they are incomplete or inaccurate.
- Do not document vendored or external code: `lib/cymon/`, the legacy firmware in `hardware/batterycasecontroller/`, or Zephyr and west workspace sources.
- Do not document Zephyr glue (`app/`, `hardware/zephyr_*`) as portable API. Describe only what the file does on its target.

## File header

Do not add or change a license or copyright header unless the user explicitly requests it or the repository requires it. If the file already has a project header, preserve its structure and update only the file-specific description when needed.

When a project header is required, replace all placeholders such as `<year>` and `file_name.h/cpp` with real values. The file description should use the `@file` tag and briefly describe the file's purpose and contents.

The repository's UNIMOC header format is below. The `SPDX-License-Identifier` line is part of it (Zephyr convention). Non-C++ sources (CMake, YAML, shell) carry the same tag as a first-line `# SPDX-License-Identifier: GPL-3.0-or-later` comment.

```
/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file actual_file_name.hpp
 *	@brief Brief description of the file's purpose and contents.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
```
### Old file header

if a file header like the below exists, it should be replaced with the new format above, and the file description should be updated to accurately reflect the file's purpose.
```
/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2026 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
```

## Header files

Use `#pragma once` for header files, after the file header and before includes. Remove an old include guard only when it is part of the requested documentation change and the replacement is safe for the repository.

## Groups and namespaces

Align Doxygen groups with the repository's existing namespaces and module structure.

- Use `@defgroup GROUP_NAME` once to define a new group, followed by its description.
- Use `@addtogroup GROUP_NAME` when adding a file or API to an existing group.
- Use uppercase names with underscores only when that matches the repository convention.
- Do not invent a new group when an appropriate existing group is available.
- Include a short `@code`/`@endcode` example only when it clarifies the public API.

## API documentation

Document public API elements with concise, accurate comments:

- Use `@brief` for the purpose and behavior.
- Use `@param` for non-obvious parameter meaning, units, ownership, or valid ranges.
- Use `@return` for non-void return values, including error or ownership semantics.
- Use `@tparam` for template parameters when their purpose is not obvious.
- Use `@note`, `@warning`, or `@pre` for important constraints and invariants.
- Use `@var` for public data members whose meaning is not self-evident.
- Use `@fn` only when needed to document a function independently of its declaration; normally place the Doxygen block directly before the declaration or definition.

Example:

```cpp
 /**
 * @brief Returns the electrical angle in radians.
 * @param position Rotor position to convert.
 * @return The electrical angle in the range [0, 2*pi).
 */
auto electrical_angle(const RotorPosition& position) -> float;
```

## Validation

Before finishing:

1. Confirm the file name, year, and any other header values are real rather than placeholders.
2. Confirm every `@param`, `@tparam`, and `@return` tag matches the declaration.
3. Confirm group names are defined exactly once and referenced consistently.
4. Run the repository's Doxygen or documentation build when available.
5. Run the relevant formatter or compile/test check if the documentation change altered C++ preprocessing or code layout.