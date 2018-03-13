
# Coding Conventions

Keep it simple. As simple as possible, but not simpler.
Emphasize readability. Code will be read many times, but only written once.

## History of this document.

* 2018/03/13 Initial version.


## Baseline

* These conventions are based on Goggle's guidlines (https://google.github.io/styleguide/cppguide.html). If nothing is stated here to the contrary, refer to the google guidelines.
* This is a guideline, not a set of commandments. There will be exceptions. Treat them as such.
* This document is subject to change.

## Attitude

There may be plenty of code violating this convention. Do not use this as justification to deviate from the style guide.
Instead, use common courtesy and try to leave a place cleaner or at least as clean than you entered it.



## Visuals

* Spaces instead of Tabs. Use four spaces for normal indentation.
* Do not indent namespaces. Almost all code is inside a namespace or even nested namespace. Indentation here wastes predious screen realestate.

## Naming Conventions

* Classnames and structs are writting in `CamelCase` (first letter capitalized).
* Method names are written in `camelCase` (first letter lower case).
* Member variables are written in `_camelCase`, i.e., lower case first letter and prefixed with an underscore.

## Type Operators and Modifiers

* The usage of  `const` is encouraged. `const` increases readability and safety of the codebase.
* Pointer and Reference Expressions:
When declaring pointers and references, prefer placing the operator adjacent to the type rather than the name. This improves consistency when using the type in other expressions.
   * Prefer `int* p` instead of `int *p`.
   * Prefer `Vector3& pos` over `Vector3 &pos`.



## Common Pitfalls

* Shared Pointers: They are not the solution to all your memory problems. Even though they are very common in this codebase, avoid using them. They are slow, cannot deal with cyclic relationships and often hinder readability, as they obscure the true ownership relationship. Use other techniques if at all possible (unique_ptr especially are fabulous). IF you need to introduce a new shared pointer, submit a description of your intend, a clear description of why this problem requires a shared pointer and a description of the ownership model that you want to use in triplicate (one signed original plus two copies of the signed original) to hendrik.skubch@rapyuta-robotics.com.

