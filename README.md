# **FRC Team #8016: 2022 FIRST Forward/Rapid React Robot Code**

## **Setup:**

To properly set up your development environment for the 2023 season, you must install VSCode and WPILib extensions (found [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)) and git (found [here](https://git-scm.com/)).

### **Cloning**

To clone this git repository, please follow the steps outlined [here](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository).

## **Repo overview:**

This repository uses an alpha -> stable -> release branch system. The alpha branch is where all new additions are tested. Once proven that the code works at least nominally, and known issues are tracked, it can be pushed to stable. In stable, code can be tuned and bugs fixed, until the code is competition ready. Once that happens, it can be pushed to release. Any code in release should be competition ready.

### **Contributing**

You must be a member of FRC team 8016 to contribute. If you're adding a feature or fixing a bug in the alpha branch, create a new branch from alpha. Make your changes there, and then open a pull request back into alpha. If you're fixing bugs or tuning values in stable or release, make a new branch from the branch you're trying to fix, make your changes, and open a pull request back into the branch you're trying to fix. Use clear, concise commit messages as outlined in [this](https://cbea.ms/git-commit/#seven-rules) guide.

When creating a new branch, please name it with a description of what the branch is for.

## **Code formatting:**

When editing code, please use these formatting conventions:

- Classes should be in UpperCamelCase
- Methods should be in lowerCamelCase
- Variables should be concise but descriptive, and written in lowerCamelCase
- Constants should be kept solely in the Constants class, and should be written in UPPERCASE_SEPERATED_BY_UNDERSCORES
- When you create a method, write a comment telling us what the method does, and if necessary, how it does it.
- When you create a constant, include units either in the name or in a comment to the right of the constant declaration.
- Bracketing: follow the conventions below:
```
public void ExampleMethod() {
    if(x == 1) {
        //Do something
    } else {
        //Do something else
    }
}
```
