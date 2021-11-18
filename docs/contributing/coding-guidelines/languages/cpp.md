# C++ coding guidelines

This document follows the convention of [RFC2119](https://datatracker.ietf.org/doc/html/rfc2119).

## Background

Autoware is mainly developed using C++ language. Since the C++ language is powerful and flexible, describing essential DOs and DON'Ts of writing C++ code is useful to keep the quality of the code base.

## Goals

This document mainly focuses on:

- describing how to avoid pitfalls of C++14 or higher, and
- to be a good reference for code reviewers and developers.

## Coding Style

To keep the code base maintainable, Autoware's C++ code must follow the following guidelines:

- [Guidelines for Naming in Autoware.Auto](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware-common-naming-guidelines.html) (TODO: replace to the Autoware.Core's document)
- [ROS 2 Code style](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)

## Code Metrics

- [Lizard](https://github.com/terryyin/lizard) is useful to measure the code metrics described below.
- [ros-metrics-reporter](https://github.com/tier4/ros-metrics-reporter) measures several code metrics automatically.

### Mt1: All functions should contain no more than 200 logical lines of code

#### Reason

Long functions might be difficult to read, understand, reuse, and maintain.

### Mt2: All functions should have a cyclomatic complexity number of 20 or less

#### Exception

Auto-generated code, or functions which implement complex algorithms.

#### References

- [Cyclomatic complexity (Wikipedia)](https://en.wikipedia.org/wiki/Cyclomatic_complexity)
- Arthur H. Watson and Thomas J. McCabe. 1996. Structured Testing: A Testing Methodology Using the Cyclomatic Complexity Metric. NIST Special Publication 500-235. ([PDF](http://www.mccabe.com/pdf/mccabe-nist235r.pdf))<!-- cspell:disable-line -->

### Mt3: All functions should have no more than 6 arguments (parameters)

#### Reason

Functions which have many arguments might indicate the design failure in the functions.

#### References

- AV Rule 101; Lockheed Martin. 2005. Joint Strike Fighter Air Vehicle C++ Coding Standards for the System Development and Demonstration Program. Document Number 2RDU00001 Rev C. ([PDF](https://www.stroustrup.com/JSF-AV-rules.pdf))

## System of Measurement

### Un1: [SI units](https://en.wikipedia.org/wiki/International_System_of_Units) and [SI derived units](https://en.wikipedia.org/wiki/International_System_of_Units) should be used in code and documents

#### Reason

[REP-103](https://www.ros.org/reps/rep-0103.html) defines the rule.

## Naming

### Nm1: For a ROS parameter and a variable which do not follow the [SI base units](https://en.wikipedia.org/wiki/SI_base_unit), a suffix such as `deg` (degree) and `ms` (millisecond) should be added to these names

### Example

`steer_limit_deg`, `vehicle_cmd_delta_time_ms`.

## Header Files

### Hd1: Unused header files must not be included

### Hd2: A header file should be added explicitly for every symbol referred in a source file

#### References

- See the [Include What You Use](https://google.github.io/styleguide/cppguide.html#Include_What_You_Use) rule in Google C++ Style Guide.

## Code Clones

### Cc1: Code clones (duplicated code) should be removed

#### Reason

Code clones are known to constitute a major source of faults.

#### References

- [Lizard](https://github.com/terryyin/lizard#code-duplicate-detector) can find code clones.
- Elmar Juergens, Florian Deissenboeck, Benjamin Hummel, and Stefan Wagner. 2009. Do code clones matter? In Proceedings of the 31st International Conference on Software Engineering (ICSE '09). IEEE Computer Society, USA, 485–495. DOI:<https://doi.org/10.1109/ICSE.2009.5070547>. arXiv:<https://arxiv.org/abs/1701.05472>. <!-- cspell:disable-line -->

## Comments

### Cm1: Commented out code should be removed

#### Reason

Commented out code might decrease maintainability.

## Build

### Bl1: All packages should be able to be built with GCC and Clang

#### Reason

A package that can only be built with either one of them might contain flaws such as undefined behavior.

#### Exception

A package depending on the external libraries which can only be built with either one of them.

#### References

- Colcon might use GCC by default. If you want to build a package with Clang, use the CMake option such as `-D CMAKE_CXX_COMPILER=clang++`.
- [Undefined Behavior Sanitizer](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html) is useful to detect undefined behavior.

## Classes

### Cl1: A member function must be made `const` if it can be made `const`

#### References

- "[Con.2: By default, make member functions `const`](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con2-by-default-make-member-functions-const)"

### Cl2: A definition of a special function should follow either "the rule of zero" or "the rule of five"

#### References

- "[C.20: If you can avoid defining default operations, do](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines.html#c20-if-you-can-avoid-defining-default-operations-do)"
- "[C.21: If you define or `=delete` any copy, move, or destructor function, define or `=delete` them all](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines.html#c21-if-you-define-or-delete-any-copy-move-or-destructor-function-define-or-delete-them-all)"
- "[C.81: Use `=delete` when you want to disable default behavior (without wanting an alternative)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c81-use-delete-when-you-want-to-disable-default-behavior-without-wanting-an-alternative)"

### Cl3: Must not define a default constructor that only initializes data members

#### References

- "[C.45: Don’t define a default constructor that only initializes data members; use in-class member initializers instead](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c45-dont-define-a-default-constructor-that-only-initializes-data-members-use-in-class-member-initializers-instead)"

### Cl4: Class data members should be initialized by member initializers if these are initialized in the constructor

#### References

- "[C.49: Prefer initialization to assignment in constructors](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c49-prefer-initialization-to-assignment-in-constructors)"

### Cl5: Inheriting constructors must be used to import constructors into a derived class that does not need further explicit initialization

#### References

- "[C.52: Use inheriting constructors to import constructors into a derived class that does not need further explicit initialization](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c52-use-inheriting-constructors-to-import-constructors-into-a-derived-class-that-does-not-need-further-explicit-initialization)"

### Cl6: A base class destructor must be either "public and virtual" or "protected and non-virtual"

#### References

- "[C.35: A base class destructor should be either public and virtual, or protected and non-virtual](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c35-a-base-class-destructor-should-be-either-public-and-virtual-or-protected-and-non-virtual)"

## Lambda Expressions

### Lm1: All variables should be captured explicitly

#### References

- "[F.54: If you capture this, capture all variables explicitly (no default capture)](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f54-if-you-capture-this-capture-all-variables-explicitly-no-default-capture)"

## Casts

### Cs1: C-style casts must not be used. Named casts must be used instead

#### References

- The compiler option `-Wold-style-cast` helps to detect C-style casts.
- "[ES.48: Avoid casts](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es48-avoid-casts)".
- "[ES.49: If you must use a cast, use a named cast](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es49-if-you-must-use-a-cast-use-a-named-cast)"

## Statements

### St1: Floating-point numbers must not be compared with `==` or `!=`

#### References

- The compiler option `-Wfloat-equal` helps to detect floating-point numbers being compared with `==` or `!=`.<!-- cspell:disable-line -->
- [Floating-point Comparison](https://www.boost.org/doc/libs/1_77_0/libs/math/doc/html/math_toolkit/float_comparison.html).

### St2: Redundant `==` or `!=` must not be added to conditions

#### References

- "[ES.87: Don’t add redundant == or != to conditions](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es87-dont-add-redundant--or--to-conditions)"

### St3: A ranged-`for` loop should be used instead of a `for` loop that iterates through all elements of a container

#### References

- "[ES.55: Avoid the need for range checking](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es55-avoid-the-need-for-range-checking)"
- "[ES.71: Prefer a range-for-statement to a for-statement when there is a choice](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es71-prefer-a-range-for-statement-to-a-for-statement-when-there-is-a-choice)"

## Declarations

### Dc1: An object must be made `const` or `constexpr` by default

#### Reference

- "[ES.25: Declare an object `const` or `constexpr` unless you want to modify its value later on](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es25-declare-an-object-const-or-constexpr-unless-you-want-to-modify-its-value-later-on)"
- "[Con.1: By default, make objects immutable](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con1-by-default-make-objects-immutable)"
- "[Con.4: Use `const` to define objects with values that do not change after construction](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con4-use-const-to-define-objects-with-values-that-do-not-change-after-construction)"

### Dc2: `using` must be used instead of `typedef`

#### Reference

- "[T.43: Prefer using over typedef for defining aliases](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#t43-prefer-using-over-typedef-for-defining-aliases)"

### Dc3: Must declare one name only per declaration

#### References

- "[ES.10: Declare one name (only) per declaration](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es10-declare-one-name-only-per-declaration)"

### Dc4: Class enum must be used instead of plain enum

#### References

- "[Enum.3: Prefer class enums over “plain” enums](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#enum3-prefer-class-enums-over-plain-enums)"

### Dc5: To return multiple values, a `struct` or `std::tuple` (`std::pair`) should be used

#### References

- "[F.21: To return multiple “out” values, prefer returning a struct or tuple](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f21-to-return-multiple-out-values-prefer-returning-a-struct-or-tuple)"

### Dc6: An argument that cannot be NULL should be passed by reference

#### References

- "[F.60: Prefer T\* over T& when “no argument” is a valid option](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f60-prefer-t-over-t-when-no-argument-is-a-valid-option)"

### Dc7: Must use smart pointers as arguments only to express lifetime semantics explicitly

#### References

- "[R.30: Take smart pointers as parameters only to explicitly express lifetime semantics](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#r30-take-smart-pointers-as-parameters-only-to-explicitly-express-lifetime-semantics)"

## Initialization

### In1: An object must be initialized before it is used

#### References

- "[ES.20: Always initialize an object](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines.html#es20-always-initialize-an-object)"

### In2: The `{}`-initializer syntax except `={}` should be used to initialize a variable

#### Exception

Must not initialize an `auto` type variable using a `{}`-initializer.

#### References

- "[ES.23: Prefer the {}-initializer syntax](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es23-prefer-the--initializer-syntax)"
- "[T.68: Use {} rather than () within templates to avoid ambiguities](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#t68-use--rather-than--within-templates-to-avoid-ambiguities)"
- "[ES.64: Use the T{e}notation for construction](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#es64-use-the-tenotation-for-construction)"

## Exception Handling

### Ex1: Exceptions must be used for error handling only

#### References

- "[E.3: Use exceptions for error handling only](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#e3-use-exceptions-for-error-handling-only)"

### Ex2: Must only throw instances of types derived from `std::exception`

#### References

- "[E.14: Use purpose-designed user-defined types as exceptions (not built-in types)](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#e14-use-purpose-designed-user-defined-types-as-exceptions-not-built-in-types)"

## Strings

### St1: C-style strings should not be used

#### References

- "[SL.str.1: Use std::string to own character sequences](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#slstr1-use-stdstring-to-own-character-sequences)"

## TODOs

- Merge with [Guidelines and Best Practices](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html).
- Add code examples.
