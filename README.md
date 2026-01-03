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

In order to make changes, a local (on your machine) copy is needed.

For most it is recommended to use **Github Desktop** to do this. If you don't have it installed, [https://desktop.github.com/download/](https://desktop.github.com/download/).
This guide is provided for both Github Desktop and Git (w/ Github CLI).

> [!NOTE]
> **Git** and **Github** are entirely seperate things. Git is version control software, Codebases on Github use git for version control. 

#### Github Desktop

<details>

2. When you open a repo a github.com, you will see a green **Code** in the top right. Click it to open cloning options.

3. Click **Open with Github Desktop**

4. A prompt will appear in the app, simply click the Clone button.

*Side Note: You can also use the repo browser in github desktop, but it's quite clunky.*

</details>

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

### Github Desktop

<details>

1. In the top of github desktop you will see **Current Branch**

2. Click the arrow to the right to open a dropdown of the existing branches

3. If you were assigned a specific branch, select it here. If you were told to create a new one, you can do so from this menu.

</details>

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

> [!IMPORTANT]
> The **Main** or **Master** branch is the default branch, and source of truth of the entire codebase. In large projects it is **write-protected**, meaning you can not directly edit it.

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

#### 1. Discarding Changes (Resetting)

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

#### 2. Undoing the Last Commit

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

#### 3. Reverting a Previous Commit

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

### K. Syncing Changes (Pushing & Pulling)

When you commit changes, they are stored **locally** on your computer. To share your work with the team, you must send those changes to the server (the **remote repository**).
Similarly, you must download changes made by others to keep your local code up to date.

#### 1. Fetching and Pulling

**Fetching** checks the server for any new changes without merging them into your code. **Pulling** actually downloads those changes and integrates them into your local branch.

#### Github Desktop

<details>

1. In the top bar, to the right of the branch selector, you will see a button labeled `Fetch Origin`, `Pull Origin`, or `Push Origin`.

2. If you see **Fetch Origin**, click it. GitHub will check the server for updates.

3. If new changes are found, the button will change to **Pull Origin**. Click it to bring those changes into your local repository.

4. If you see **Push Origin**, your local repository is already up to date with the server, and you have local commits ready to be sent up.

</details>

#### Terminal

<details>

To download and integrate changes from the remote repository, run:

```bash
git pull
```

</details>

#### 2. Handling Merge Conflicts

If you and another person edit the **same line** in the same file, Git won't know which version to keep. This results in a **merge conflict**. You must manually choose which code to preserve.

#### Github Desktop

<details>

GitHub Desktop provides a clear interface when a conflict occurs. A notification bar will appear for each conflicted file, allowing you to choose whether to:

- Keep Your Changes (Local)

- Keep Their Changes (Remote)

- Keep Both (Rarely used, but merges both versions together)

</details>

#### Terminal

<details>

If `git pull` results in a conflict, Git will mark the affected files. Open the file in your code editor (like VS Code). You will see markers like this:

```
<<<<<<< HEAD
(Your local changes)
=======
(Changes from the server/remote)
>>>>>>> BRANCH-NAME
```

To resolve the conflict:

1. Delete the code you don't want to keep.

2. Remove the <<<<<<<, =======, and >>>>>>> markers.

3. Save the file and commit the resolution.

*Note: In VS Code and similar editors, buttons like "Accept Current Change" or "Accept Incoming Change" will appear above these markers to handle this automatically.*

</details>

#### 3. Pushing Changes

Once your local changes are committed and your repository is synced with the latest updates from others, you need to **push** your work to the server.

#### Github Desktop

<details>

1. Click Push Origin in the top bar.

2. If the button says Fetch Origin, click it first to ensure there are no remote changes you missed.

3. If a conflict occurs during the push, refer to the "Merge Conflicts" section above.

</details>

#### Terminal

<details>

To send your committed changes to the server, run:

```bash
git push
```

</details>

> [!IMPORTANT]
> Always **pull** before you **push**. Both Github Desktop and Git will warn you when you push, but when working with a team it is a good habit to have.
