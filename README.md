# BadRobots 2026

This is the codebase used for 2026, and the first iteration of the team using `Python` as the primary language.

## Getting Started

This guide serves as a starting point for contributing to the robot code. Since we are using Python, the barrier to entry is lower than in previous years. Please note that setup instructions differ slightly between Linux and Windows.

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
4. A prompt will appear in the app, simply click the Clone button.

#### Terminal

<details>

To clone via the terminal, you can use git:

```bash
git clone git@github.com:BadRobots1014/pybase-2026.git
```

Or the GitHub CLI:

```bash
gh repo clone BadRobots1014/pybase-2026
```

</details>

### D. Branching

A codebase has branches, which are (essentially) isolated versions of code that others can work on and see the work being done **without messing up everyone elses work**. With numerous people touching the code, we want to avoid disrupting everyones work when possible.

1. In the top of github desktop you will see **Current Branch**
2. Click the arrow to the right to open a dropdown of the existing branches
3. If you were assigned a specific branch, select it here. If you were told to create a new one, you can do so from this menu.

#### Terminal

<details>

If the branch already exists, run:

```bash
git switch <branch>
```

If you need to create a new branch:

```bash
git switch -c <new_branch>
```

</details>

### E. Access the code

From GitHub Desktop, navigate to the Repository menu in the top left (or the button in the main view) and select **Open in Visual Studio Code**.

### F. Install Dependencies

For the code to function, we need to install specific libraries. It is best practice to create a **virtual environment** (venv) to isolate these dependencies. You can run the following commands in the VS Code terminal.

#### Windows

<details>

1. Create the virtual environment:

```bash
py -3.13 -m venv venv/
```

2. Activate the environment. You **must do this every time** you open VS Code. You will see a (venv) prefix in your terminal when it is active.

```bash
venv\Scripts\activate
```

3. Install the required robot libraries:

```bash
pip install robotpy[all]
```

</details>

#### Linux

<details>

Create the virtual environment:

```bash
python -m venv venv/
```

2. Activate the environment. You **must do this every time** you open VS Code. You will see a (venv) prefix in your terminal when it is active.

- bash/zsh:

```bash
source venv/bin/activate
```

- fish:

```fish
source venv/bin/activate.fish
```

Install the required robot libraries:

```bash
pip install robotpy[all]
```

</details>

### G. Writing code

As of this moment we don't have a dedicated tutorial or reference. Those will be added at a later date. For now, consult with experienced team members and mentors.

### H. Testing

Once you have written code, we must ensure it behaves as expected. Tests help identify **unintended, silent errors** that your code editor might not catch.

```bash
robotpy test
```

- All Green: You are all set!
- Red 'F': A test failed. Try your best to solve it. If you cannot figure it out, speak with an experienced member or mentor.

### I. Committing

A commit is a record of a change. When you edit code, you must record it so others know what you did.

When you make changes, they will appear in GitHub Desktop. **Green** text indicates additions, while **red** text indicates deletions.

The most important part of a commit is the title. In a perfect world, we would use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/#summary), but given our rapid pace, we focus on four simple rules:

1. **Use Present Tense Actions**: Follow the standard git practice.
   - Good: "make xyzzy do frotz"
   - Bad: "I changed xyzzy to do frotz" or "Made xyzzy do frotz"

2. **Use Descriptive Titles**: We need to understand the change at a glance.
   - Examples: "Create README.md", "Change shooting angle from degrees to radians", "Fix IdealStartingState arguments".

3. **Keep Titles Short**: Do not write a paragraph in the title. If you need more detail, add it to the description field.

4. **Commit Often**. This just comes down to experience. I recommend you do so after you make a change to anything (e.g. updating a function, changing constants).

Once you have written your title, click the Commit button in the bottom left. It is **better to write too many commits than too few**. It makes tracking changes significantly easier.

#### Terminal

<details>

1. Stage your changes. This tells git to include all new, modified, or deleted files in the next commit.

```bash
git add -A
```

2. Commit the changes.

```bash
git commit -m "[title]"
```

_Note: If you have modified existing files but haven't created new ones, you can combine these steps using `git commit -am "[title]"`, but `git add -A` is safer for beginners._

</details>

### J. Undoing changes

At some point, you will need to undo a **previous commit**, a **changed file**, or **all work since the last commit**. This process is straightforward as long as you make it a habit to **commit often**.

> [!NOTE]
> The HEAD is the most recent commit on your current branch. It updates automatically every time you commit. Think of it as a "You Are Here" marker on your timeline.
> It is most relevant if you're using terminal commands.

#### Discarding Changes (Resetting)

Resetting is the most common way to undo work. It discards any changes made since the last commit, effectively acting like a "Ctrl+Z" for your entire project.

#### GitHub Desktop

<details>

1. Go to the **History** tab (located above your recent changes).

2. Right-click the most recent commit.

3. Select **Reset to Commit**.

</details>

#### Terminal

<details>

To discard all uncommitted changes in your working directory:

git reset --hard

</details>

#### Undoing the Last Commit

If you committed too early or made a typo in your commit title, you can "undo" the commit. This removes the commit record but **keeps your code changes** as uncommitted work so you can keep editing.

#### GitHub Desktop

<details>

Click the **Undo** button located immediately below the Commit button in the bottom left.

</details>

#### Terminal

<details>

To move the HEAD back one commit while keeping your changes in the text editor:

git reset --soft HEAD~1

</details>

#### Reverting a Previous Commit

Reverting creates a **new commit** that does the exact opposite of a previous commit. This is the safest way to undo work that has already been shared with others, as it does not delete history.

#### GitHub Desktop

<details>

1. Go to the **History** tab.

2. Right-click the specific commit you want to reverse.

3. Click **Revert changes in commit**.

</details>

#### Terminal

<details>

1. Find the commit ID (a string of numbers and letters) by viewing the log:

```bash
Find the commit ID (a string of numbers and letters) by viewing the log:
```

2. Run the revert command:

```bash
git revert <commit-id>
```

</details>

> [!WARNING] 
> Reverting can be a "footgun" if the code has changed significantly since that commit was made, as it may cause merge conflicts. Use this carefully.
