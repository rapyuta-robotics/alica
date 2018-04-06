
# Coding Conventions

Keep it simple. As simple as possible, but not simpler.
Emphasize readability. Code will be read many times, but only written once.

## History of this document.

* 2018/03/13 Initial version.
* 2018/04/05 Merged with Carpe-Noctem-Cassel code style.


## Baseline

* These conventions are based on Goggle's guidelines (https://google.github.io/styleguide/cppguide.html). If nothing is stated here to the contrary, refer to the google guidelines.
* This is a guideline, not a set of commandments. There will be exceptions. Treat them as such.
* This document is subject to change.

## Attitude

There may be plenty of code violating this convention. Do not use this as justification to deviate from the style guide.
Instead, use common courtesy and try to leave a place cleaner or at least as clean than you entered it.

If your team shares development PCs, please note that uncommitted changes on those public PCs or Laptops are deleted w/o further research. Thus, keep them clean.

## Visuals

* Spaces instead of Tabs. Use four spaces for normal indentation.
* Do not indent namespaces. Almost all code is inside a namespace or even nested namespace. Indentation here wastes precious screen real estate.
* If possible use the common clang-format file that resides aside of these Coding Guidelines and integrate it into your workflow/IDE.

## General Remarks

* Write C++11 not C and avoid utilising Boost when this needed feature is in C++11, already.
* Try to write Object-Oriented code.
* Avoid code duplication at all costs.

## Naming Conventions

* Classnames and structs are written in `CamelCase` (first letter capitalized).
* Method names are written in `camelCase` (first letter lower case).
* Member variables are written in `_camelCase`, i.e., lower case first letter and prefixed with an underscore.

## Type Operators and Modifiers

* The usage of  `const` is encouraged. `const` increases readability and safety of the codebase.
* Pointer and Reference Expressions:
When declaring pointers and references, prefer placing the operator adjacent to the type rather than the name. This improves consistency when using the type in other expressions, such as templates or casts.
   * Prefer `int* p` instead of `int *p`.
   * Prefer `Vector3& pos` over `Vector3 &pos`.
   
## Includes

* in header files do forward declaration instead of includes whenever possible: It compiles faster and avoids cycling include dependencies.
* use `"Bla.h"` for header files that are inside your local include folder
* use `<Bla.h>` for header files that are outside your local include folder
* sort the header files from local to general:
    1. include your own header: `#include "MyClass.h"`
    2. include header from your include folder: `#include "UDPConnection.h"`
    3. include from our own projects: `#include <engine/model/Plan.h>`
    4. include from third party libraries: `#include <ros/ros.h>`
    5. include system library headers: `#include <vector>`
* sort the includes alphabetically in each section and keep a newline between those sections (this is done automatically by clang-format)
* don't include with relative paths, e.g., `#include "../../folder/header.h"`

## Namespaces

- always use at least one namespace
- As a rule of thumb - Use one namespace per repository:
    * supplementary: supplementary[::nested_namespace]
    * alica: alica[::nested_namespace]

## Header

* use `#pragma once` instead of include guards whenever possible: Nowadays all common compilers support that feature and it facilitates refactoring and avoids include guard clashes.
* have at most one 'public:', 'protected:', and 'private:' section and list them in this order
* in each section place functions first, then member variables
* especially place constructors and destructor at first

## Common Pitfalls

* Shared Pointers: They are not the solution to all your memory problems. Even though they are very common in this codebase, avoid using them. They are slow, cannot deal with cyclic relationships and often hinder readability, as they obscure the true ownership relationship. Use other techniques if at all possible (unique_ptr especially are fabulous). IF you need to introduce a new shared pointer, discuss your intent, including why this problem requires a shared pointer and the ownership model that you want to use with some senior developer. 
   * Especially Hendrik (hendrik.skubch@rapyuta-robotics.com) will help you to avoid shared_ptr wherever possible.

* `using` and `using namespace`: 
   * Never write using in a header file. Ever. 
   * Don't write `using` before an #include.
   * Never write `using namespace XYZ;` write `using XYZ::classname;` instead.

