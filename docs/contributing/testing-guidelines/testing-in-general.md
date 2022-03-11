# Testing in General

This article explains the importance of testing.

## Importance of testing

Dynamic and static testing methods make Autoware reliable and robust, helping us to perform anomaly detection.
Through testing in Autoware, we can estimate the number of Heisenbugs, and find and eliminate [undefined behaviors](https://blog.regehr.org/archives/1520) for which C++ languages are known for.

Dynamic analysis, simply called “testing” as a rule, means executing the code while looking for errors and failures.

Static analysis means inspecting the code to look for faults. Static analysis is using a program (instead of a human) to inspect the code for faults.

There are also formal verification methods (see the [book](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704), Chapter 15); note that the topics will not be covered in this document.

## Testing in Autoware

This section introduces various types of tests that are run both manually and automatically.

When creating a new node, be sure to include the following test, which will be explained later:

- For core package
  - Unit testing with `gtest`
- For node package
  - Smoke testing with `smoke_test`
  - Interface testing with `interface_test`
  - Integration testing with `launch_testing`

### Style / linter tests

Some examples of tools used for style and linting are
[cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint),
[uncrustify](https://github.com/uncrustify/uncrustify).

Tests using the tools above allow Autoware to follow C++ style guides which results in uniform, easy to read code.

### Static code analysis

The [Cppcheck](https://github.com/danmar/cppcheck) tool is used for applications written in Autoware.

Static code analysis tools detect the following types of errors:

- API usage errors
- Best practice coding errors
- Buffer overflows
- Build system issues
- Class hierarchy inconsistencies
- Code maintainability issues
- Concurrent data access violations
- Control flow issues
- Cross-site request forgery (CSRF)
- Cross-site scripting (XSS)
- Deadlocks
- Error handling issues
- Hard-coded credentials
- Incorrect expression
- Insecure data handling
- Integer handling issues
- Integer overflows
- Memory—corruptions
- Memory—illegal accesses
- Null pointer dereferences
- Path manipulation
- Performance inefficiencies
- Program hangs
- Race conditions
- Resource leaks
- Rule violations
- Security best practices violations
- Security misconfigurations
- SQL injection
- Uninitialized members

### Unit tests

Unit-testing is a software testing method by which individual units of source code are tested to determine whether they are fit for use.

The tool used for unit testing in Autoware is [gtest](https://github.com/google/googletest).

`gtest` is a tool for unit testing all source code in core packages.
For node testing, consider using `launch_testing`. Documentation is available in [Integration testing](integration-testing.md).

Unit tests that are common to all nodes are available in `autoware_testing`.
Please refer to the following design documentation for usage.
[Link](https://github.com/autowarefoundation/autoware.universe/blob/main/common/autoware_testing/design/autoware_testing-design.md)

### Integration tests

In integration-testing, the individual software modules are combined and tested as a group.
Integration testing occurs after unit testing.

While performing integration testing, the following subtypes of tests are written:

1. Smoke testing
2. Interface testing
3. Node integration testing

#### Smoke tests

[Smoke test](https://en.wikipedia.org/wiki/Smoke_testing_(software)) has several meanings. In this section, smoke test is to perform the following evaluations.

- Smoke test ensures that the node can be
  1. launched with its default configuration and doesn't crash.
  2. shut down through a `SIGINT` signal with the corresponding process return code.

Autoware provides a smoke testing framework that tests all nodes uniformly.

#### Interface tests

Interface test ensures that a node consumes/produces the desired output in line with the high-level documentation of its design document.
It helps to keep the docs up to date and to prevent users from having to dig through the source code to find the topic names/types.

Autoware provides a interface testing framework that tests all nodes uniformly.

#### Node integration tests

Node integration tests (or simply referred to as an integration tests) is a test to evaluate the interaction of a single Node or multiple Nodes.
Integration testing verifies interactions of nodes that would not be found in a unit tests.

The tool used for node integration testing in Autoware is [launch_testing](https://github.com/ros2/launch/tree/master/launch_testing).

### Memory tests

Memory tests allow the detection of unwanted calls to memory management APIs, such as:

- `malloc`
- `calloc`
- `realloc`
- `free`

For more details on memory tests see the [memory testing](https://github.com/osrf/osrf_testing_tools_cpp#memory_tools) tool.

### In-vehicle tests

Regular in-vehicle testing is performed as part of ODD development and demonstrations. These tests validate Autoware in a realistic autonomous vehicle product.

### References

1. [colcon](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) is the tool of choice for building and running tests
2. [ament_cmake](https://github.com/ament/ament_cmake) is useful to specify tests in CMake
3. Chris Hobbs' [_Embedded Software Development for Safety Critical Systems_](https://www.amazon.com/Embedded-Software-Development-Safety-Critical-Systems/dp/1498726704), describes tests necessary for code running in safety critical environments
4. [ISO 26262 standard](https://www.iso.org/standard/51362.html) part 6 prescribes how to test code in automotive systems
5. [SQLite](https://www.sqlite.org/testing.html) is a software project that has an impressive and thoroughly described testing system
