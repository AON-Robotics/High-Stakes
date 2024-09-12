# 😺 GitHub Actions ⚒️

[GitHub Actions](https://github.com/features/actions) is a tool that automates
workflows triggered by GitHub events or even schedule them. What this means in
practice is that GitHub can run some commands for us within their own servers
in order to reduce the amount of work we have to do.

What motivated me most to integrate GitHub Actions into our project is that we
can automatically lint our code using
[`cpplint`](https://github.com/cpplint/cpplint). This helps reviewers focus less
on code style and instead focus on more important aspects of code, like its
semantics. However, there are additional benefits to take advantage of. For
example, we could finally integrate our GitHub repo with YouTrack tickets by
hitting the YouTrack API when Pull Requests are merged (for example), or tie
tickets to particular Pull Requests when they are first opened.
