# BadRobots 2026

This is the codebase used for 2026, and the first iteration of the team using `Python` as the primary language.

## Getting Started

This guide serves as a strong starting point for learning how to contribute. Since it is python, contributing is a lot easier than it used to be. The instructions for Linux and Windows are slightly different so keep that in mind.

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

#### Terminal

<details>

For cloning in the terminal you can either use git:

```bash
git clone git@github.com:BadRobots1014/pybase-2026.git
```

or the Github CLI

```bash
gh repo clone BadRobots1014/pybase-2026
```

</details>

### D. Branching

A codebase has branches, which are (essentially) isolated versions of code that others can work on and see the work being done **without messing up everyone elses work**. With numerous people touching the code, we want to avoid disrupting everyones work when possible.

1. In the top of github desktop you will see **Current Branch**
2. Click the arrow to the right to open a drop down of the existing branches
3. If you were told to create a branch, do so here. Otherwise, use the branch you were asked to.

#### Terminal

<details>

If the branch already exists, you simply need to run one command:

```bash
git switch <branch>
```

Or if you need to create a new branch:

```bash
git switch -c <new_branch>
```

</details>

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

Now you need to install the dependencies to this environment so the code will actually work

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

- bash/zsh:

```bash
venv/bin/activate
```

- fish:

```fish
venv/bin/activate.fish
```

Now you need to install the dependencies to the environment so the code will actually work

```bash
pip install robotpy[all]
```

</details>

### G. Writing code

As of this moment we don't have a dedicated tutorial or reference. Those will be added at a later date. For now, consult with experienced team members.

### H. Committing

A commit is a record of change. When you edit the code, you need to record it
so others know what you did. Github Desktop makes this very easy.

When you make a changes to the code, you will see them appear in Github Desktop. **Green** text means you added something, **red** text means you deleted something.

**The most important part** of a commit is the title. It is what shows up next to your commit to tell others what you did. In a perfect world we would use [Convential Commits](https://www.conventionalcommits.org/en/v1.0.0/#summary), but with how rapidly we change the code it's too restrictive. So instead there are four rules we focus on:

1. **Commits are present tense actions**. It's a practice that comes from git itself. To be consistent with standard practices, we will do the same.
   e.g. "make xyzzy do frotz" instead of "[This patch] makes xyzzy do frotz" or "[I] changed xyzzy to do frotz".

2. **Use Descriptive titles**. Often times we don't have time to look through every single commit to see what changed, so we need to be able to glance through the titles to get an idea of what changed.
   e.g. "Create README.md", "Change shooting angle from degrees to radians", "Update drivetrain speed limit", "Fix IdealStartingState arguments".

3. **Keep titles short**. A paragraph as a title is incredibly difficult to read. It is possible to be detailed and concise in a sentence. If you feel more detail is needed, add it to the description.

4. **Commit Often**. This just comes down to experience. I recommend you do so after you make a change to anything. Update a function? Commit. Update constants? Commit. **It's better to write to many commits than to few**. It makes looking back at your changes a lot easier. Trust me on this one.

Once you have the title and you want to record it, just click the commit button in the bottom left.

#### Terminal

<details>

If you create or delete a file, you need to tell git to include it. The easiest way is to tell it to include all new / old files in the codebase through `-A`. This is referred to as **staging** changes (preparing them to be recorded).

```bash
git add -A
```

Then you have to **commit** those changes. For simplicity sake we will just add a title to the the commit.

```bash
git commit -am "[title]"
```

`-a` tells git that for all files, use the title passed after `-m`. It gets combined into -am for convience. If you feel you need to add a description, don't include `m`.

</details>

### I. Testing

That's all you need to start writing code. In order to make sure the robot still works we need to run tests.

```bash
robotpy test
```

All commands we need to run and test the code are done through the **robotpy** library.

The goal is to make sure all tests are passing. It means the robot _(hopefully)_ won't die in the middle of a match, assuming the team remembers to write proper test. If you need help getting them to pass, speak with one of the experienced members.
