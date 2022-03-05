# C++

!!! warning

    Under Construction

### C++ Resources

- [cppreference.com](https://en.cppreference.com/w/)
- [C++ Core Guidelines](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)

### Cross-platform Compatibility

It is preferred to use cross-platform solutions for system-level function calls whenever possible. While the C++ standard library should be used for as many tasks as possible, some functions (such as `std::filesystem`) are not available in C++14 in cross-platform implementations. This usually means utilizing libraries like [`asio`](https://think-async.com/Asio/index.html) for networking tasks and [`a std::filesystem shim`](https://github.com/gulrak/filesystem) for filesystem navigation is preferred to creating platform-specific implementations.

### Formatting {#contributors-guidelines-formatting}

Autoware.Auto follows ROS recommendations for code style and formatting. See the [Coding Style and Language Versions entry for C++](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#id3) or the [Coding Style and Language Versions entry for Python](https://index.ros.org/doc/ros2/Contributing/Code-Style-Language-Versions/#python) for more information. We enforce these guidelines using linters provided with `ament` as far as possible. All packages should have the following in their `package.xml` files:

```{xml}
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```

In addition, the following should be in the package's `CMakeLists.txt` (extended with other tests):

```{cmake}
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

In CI, merge requests fail if they introduce improperly formatted code. To avoid that, format the C++ code locally with

```{bash}
ament_uncrustify --reformat file.cpp         # update single file in place
ament_uncrustify --reformat path/to/pkg_foo  # update all C++ source files in package
```

With the above CMake setup, run all linters together with all other tests of a package as described in the [Running Tests](#contributors-guidelines-run-tests) section or run a specific linter; e.g.,

```{bash}
ament_cpplint path/to/pkg_foo
```

Tools such as CLion can parse the output of the previous command and provide fast navigation to offending lines in the code.

#### Using ament_clang_format

`ament_uncrustify --reformat` is able to format the code to a degree but its results are generally not enough to pass `ament_cpplint`. To automate the process, `ament_clang_format` can be used like:

```{bash}
AutowareAuto $ ament_clang_format --config .clang-format --reformat file.cpp
```

The configuration is stored in the text file `.clang-format` in the base directory of a source checkout of Autoware.Auto.

`ament_clang_format` is available in ADE by default. When working outside of ADE, install it e.g. with

```{bash}
sudo apt install ros-${ROS_DISTRO}-ament-clang-format
```

A way to use all these three tools is as follows:

1. `ament_clang_format --config AutowareAuto/.clang-format --reformat file.cpp`
1. `ament_uncrustify --reformat file.cpp`
1. `ament_cpplint file.cpp`
1. Fix all reported errors
1. Repeat the previous steps until `ament_uncrustify` and `ament_cpplint` give no more errors.

##### Order of header includes

The recommended order of `#include` directives is based on the [google styleguide](https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes)

1. corresponding file: from `bar.cpp` in package `foo`, that's `#include "foo/bar.hpp"`
1. C system headers; e.g. `#include <stdint.h>`
1. C++ system headers; e.g. `#include <vector>`
1. headers from this or other packages; e.g. `#include "pkg/baz.hpp"`
1. message headers; e.g. `#include "foo/msg/bar.hpp"`

with headers in each group sorted alphabetically and groups separated by a single blank line.

In order for automatic grouping and sorting to work:

1. Use `#include "foo/bar.hpp"` for headers from the same and other packages
1. Use `#include <vector>` with **angle brackets only for C and C++ system headers**
1. Invoke `ament_clang_format` like explained above.

The resulting order should satisfy `ament_cpplint`.

!!! note

    If cpplint complains about the order of headers, ensure that the delimiters match the above rules. Example error:

    ```console
    Found C system header after C++ system header.
    ```
