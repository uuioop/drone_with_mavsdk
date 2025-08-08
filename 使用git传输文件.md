# 添加用户与邮箱
`git config --global user.name "你的用户名"
`git config --global user.email "你的邮箱"
### 添加文件
- 先克隆然后添加文件
- 在根目录运行`git add .
- 使用`git status`进行检查
- `git commit -m` "提交说明"
- `git push origin main`
### 创建新仓库
`cd your-local-project`
`git init                # 初始化为 Git 仓库`
`git remote add origin https://github.com/yourname/your-repo.git`



提交新文件需要遵循一个标准的\*\*“拉取-提交-推送”\*\*工作流。
这个流程可以避免 `rejected` 错误，并确保你的本地仓库始终与远程仓库保持同步。
### 提交工作流（推荐）

假设你正在 `main` 分支上工作。每次要提交新的内容或修改时，请执行以下步骤：

#### 步骤一：拉取最新代码（防止冲突）

在开始任何新工作或准备提交前，养成先拉取远程最新代码的习惯。这可以让你在第一时间知道远程仓库是否有更新，避免与他人的修改发生冲突。

```bash
git pull origin main
```

  * 如果有远程更新，Git 会自动合并到你的本地分支。
  * 如果没有，Git 会提示你 `Already up to date.`。
  * 如果出现 `fatal: 需要指定如何调和偏离的分支`，请使用我们之前讨论的命令：`git pull --rebase`。

#### 步骤二：进行本地修改

现在你可以自由地修改文件、添加新文件或删除文件。

#### 步骤三：暂存并提交修改

当你完成了本次的修改内容后，需要将这些修改暂存并提交到你的本地仓库。

1.  **暂存所有修改**：
    ```bash
    git add .
    ```
2.  **创建提交**：
    ```bash
    git commit -m "你的提交信息"
    ```
    （请用简洁明了的语言描述本次修改的内容）

#### 步骤四：推送到远程仓库

最后，将你的本地提交推送到 GitHub 的远程仓库。

```bash
git push origin main
```

如果你在**步骤一**中已经成功拉取了最新的代码，那么这一步通常会顺利完成。

-----

### 完整示例

如果你要修改一个文件并提交：

```bash
# 1. 切换到工作目录
cd ~/drone_with_mavsdk

# 2. 拉取远程最新代码
git pull origin main

# 3. 在编辑器中修改你的文件...

# 4. 暂存所有修改
git add .

# 5. 提交到本地仓库
git commit -m "feat: Add new offboard navigation logic"

# 6. 推送到远程仓库
git push origin main
```

通过遵循这个“**`pull` -\> `add` -\> `commit` -\> `push`**”的循环，你就可以高效且安全地管理你的代码版本了。
