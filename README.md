# BadRobots 2026

---

This is the codebase used for 2026, and the first iteration of the team using `Python` as the primary language.

## Contributing

---

Since it is python, contributing is a lot easier than it used to be. The instructions for Linux and Windows are slightly different so keep that in mind.

### A. Download Python

1. We are using a specific version of Python (Version `3.13.x`)
2. Download at https://www.python.org/downloads/release/python-3139/ or from your package manager

### B. Download Visual Studio Code

1. We aren't restricted to specific tools to edit the code this year, but VSCode is recommended.
2. Download at https://code.visualstudio.com/ or from your package manager

### C. Clone the Repository

1. For most it is recommended to use **Github Desktop**. If you don't have it installed, https://desktop.github.com/download/
2. Click the green **Code** in the top right.
3. Click **Open with Github Desktop**
4. It will open a screen to clone, just click the **Clone** button and it will open

### D. Branching

A codebase has branches, which are (essentially) isolated versions of code that others can work on and see the work being done **without messing up everyone elses work**. With a 10+ people touching the code, we want to avoid disrupting everyones work when possible.

1. In the top of github desktop you will see **Current Branch**
2. Click the arrow to the right to open a drop down of the existing branches
3. If you were told to create a branch, do so here. Otherwise, use the branch you were asked to.

### E. Access the code

From Github Desktop, in the top left there will be a tab that says **File** with a dropdown that says **Open in Code**

### F. Install Dependencies

You can run all of these commands in the terminal provided in VSCode to make things most convenient.

#### Windows

<details>

The code requires dependencies in order to function. The best way is to create a virtual environment for these to live in.

```bash
py -3.13 -m venv venv/
```

**Every time** you open VSCode you need to tell it where to find all the dependencies.
You will then get a prefix **(venv)** at the start of your terminal to tell you it is working.

```bash
venv\Scripts\activate
```

Now you need to install the dependencies to environment so the code will actually work

```bash
pip install robotpy[all]
```
</details>

#### Linux

<details>

The code requires dependencies in order to function. The best way is to create a virtual environment for these to live in.
For anyone experienced,

```bash
python -m venv venv/
```

**Every time** you open VSCode you need to tell it where to find all the dependencies.
You will then get a prefix **(venv)** at the start of your terminal to tell you it is working.

* bash/zsh:

```bash
venv/bin/activate
```

* fish:

```fish
venv/bin/activate.fish
```

Now you need to install the dependencies to environment so the code will actually work

```bash
pip install robotpy[all]
```
</details>

### G. Testing Code

That's all you need to write code. In order to make sure the robot still works we need to run tests.

```bash
robotpy test --isolated
```

All commands we need to run and test the code are done through the **robotpy** library. The `--isolated` tag just covers some annoying edge cases that causes problems.

The goal is to make sure all tests are passing. It means the robot *(hopefully)* won't die in the middle of a match, assuming the team remembers to write proper test. If you need help getting them to pass, speak with one of the experienced members.

---
