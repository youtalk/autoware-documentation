# Quality Criteria

This document describes the quality criteria for the `Autoware.Core`.

## Guidelines for General Code Development

To assure the quality of automated driving software, there are some requirements that are listed here:

1. Reliability. The software should be reliable.
2. Documentation. The documentation should be complete and accurate. This helps the users to understand the software.
3. No bugs. The software should not have serious bugs. e.g. The software should not have bugs that cause the vehicle to crash.
4. Anomaly detection. The software should detect anomalies.

To achieve the above, the following guidelines are recommended:

1. SaFAD
2. SOTIF

We referring the above guidelines and created the Quality Criteria.

## License and Copyright

Every file should have a license header. See the License section in [CONTRIBUTING.md](TODO: add CONTRIBUTING.md) for more information.

## Code Coverage

There is no clear target number for code coverage criteria. Make sure that effective unit tests are written during the review process. Also, always add unit tests for new code. This approach will ultimately lead to better code coverage.

## Coding Style

We use following coding style:

1. [Guidelines for Naming in Autoware.Auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware-common-naming-guidelines.html)
2. [ROS2 Code style and language versions](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
3. [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
4. [PEP 8](https://www.python.org/dev/peps/pep-0008/)

If there is a conflict between rules, the higher-level rule will be used.

## Code Metrics

- The number of lines for a function should be less than 200.
- The Cyclomatic complexity of a function should be less than 20.
- The argument count of a function should be less than 6.

## Cross-platform Compatibility

It is preferred to use cross-platform solutions for system-level function calls whenever possible. While the C++ standard library should be used for as many tasks as possible, some functions (such as `std::filesystem`) are not available in C++14 in cross-platform implementations. This usually means utilizing libraries like `asio` for networking tasks and a `std::filesystem shim` for filesystem navigation is preferred to creating platform-specific implementations.

## Documentation

In order to make the purpose and interface of all packages clear, there are a few rules for writing documentation. For more information, see [Documentation Guidelines](TODO: Add documentation guidelines).
