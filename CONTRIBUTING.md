# Contributing Guidelines

Thank you for your interest in contributing to `imu-ros2`. Whether it's a bug
report, new feature, correction, or additional documentation, we greatly value
feedback and contributions from our community.

Please read through this document before submitting any issues or pull requests
to ensure we have all the necessary information to effectively respond to your
bug report or contribution.


**Table of Contents:**
- [Contributing Guidelines](#contributing-guidelines)
  - [How to Contribute](#how-to-contribute)
    - [Contributing source code](#contributing-source-code)
      - [How to Contribute Source Code](#how-to-contribute-source-code)
      - [Pull Request Rules](#pull-request-rules)
    - [Bug reports](#bug-reports)
      - [Before Submitting a Bug Report](#before-submitting-a-bug-report)
      - [How to Submit a Good Bug Report](#how-to-submit-a-good-bug-report)
    - [Feature Requests](#feature-requests)
      - [Before Submitting an Enhancement Suggestion](#before-submitting-an-enhancement-suggestion)
      - [How to Submit a Good Enhancement Suggestion](#how-to-submit-a-good-enhancement-suggestion)
    - [Contributing documentation](#contributing-documentation)
      - [Documentation Best Practices](#documentation-best-practices)
  - [Becoming a Trusted Committers](#becoming-a-trusted-committers)
  - [Licensing](#licensing)
  - [Resources:](#resources)


## How to Contribute

### Contributing source code

We follow a development process designed to reduce errors, encourage collaboration, and make high quality code. Review the following to get acquainted with this development process.


#### How to Contribute Source Code

- **Create a fork of this repository.** This will create your own personal copy of the package. All of your development should take place in your fork. If you are not sure how to do this, check out [GitHub help](https://help.github.com/en/github/getting-started-with-github/fork-a-repo)

    -  Create a create a remote pointing to the [upstream remote repository](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/configuring-a-remote-for-a-fork).

  - **Work out of a new branch**, one that is not a release `main` branch, ideally
    a `develop` branch targeted for the currently supported ROS distribution, such as
    `humble-develop`. Remember to periodically rebase to have the latest code available.

- **Write your code** and remember to:
  - **Always** keep your branch updated with the original upstream.
  - **Always** [sign-off you commits](#pull-request-rules).
  - Look at the existing code and try to maintain the existing style and pattern as much as possible.

- **Resolve compiler warnings** or at least make sure you code does not add new ones.

- **Submit a pull request to the upstream repository** following the [PR rules](#pull-request-rules).

- **Check Continuous Integration (CI):** the moment you submit a pul request, a few jobs will be started which try to compile the code and run automated tests. Pay attention to any CI failures reported in the pull request, and stay involved in the conversation. On the Github UI, checks can be:
    -  ✅: Passed, all good!
    -  🟡: Pending, results haven't been received yet.
    -  ❌: Failed, something is wrong.

#### Pull Request Rules

- Commit message includes a `Signed-off-by: [name] < email >` to the commit
  message. This ensures you have the rights to submit your code, by agreeing
  to the [Developer Certificate of Origin](https://developercertificate.org/).
  If you can not agree to the DCO, don't submit a pull request, as we can
  not accept it.
    - DCO is a declaration of ownership, basically saying that you created the contribution and that it is suitable to be covered under an open source license (not proprietary).

    - If your `user.name` and `user.email` configurations are set up in git, then you can simply run `git commit -signoff` to have your signature automatically appended.

- Commit should be **atomic**, meaning it should do one thing only. A pull request may contain multiple commits if necessary to fix a bug or implement a feature.
- Commits should have good commit messages. Check out [The git book](https://git-scm.com/book/en/v2/Distributed-Git-Contributing-to-a-Project) for some pointers and tools to use.
- Typically, the title of the commit should have the path to the changed `dir/[file]`, and then explaining in a few words what has been done, like: `ci: add style check workflow for cpp files`
- Write a concise PR description, containing all the needed details.
- Pull requests will be merged only after they have been reviewed, tested and approved by the [code owners](./.github/CODEOWNERS).





### Bug reports


#### Before Submitting a Bug Report

Before creating a new issue, please search the **repository's open** and **recently closed** issues to see if the same or a similar problem has already been reported. If the issue exists and is still open, add a comment to that issue. Otherwise, feel free to create a new one.


#### How to Submit a Good Bug Report

To help us resolve issues effectively, follow these guidelines when creating a bug report:

1. **Use a clear and descriptive title** to summarize the problem.
2. **Describe the exact steps to reproduce the issue** in detail. Provide enough information for someone to follow your steps exactly.
3. **Include specific examples** such as links to files, projects, or copy-pasteable snippets that demonstrate the issue.
4. **Describe the observed behavior** after performing the steps and explain what the problem is with that behavior.
5. **Explain the expected behavior** and why it differs from the observed behavior.
6. **Attach screenshots or GIFs** to visually demonstrate the issue whenever possible.
7. **If the issue wasn’t triggered by a specific action,** describe what you were doing before the problem occurred and include relevant details.

Provide more background by answering the following questions:

- **Did the issue start recently** (e.g., after updating to a new version) or has it always been present?
- If recent, **can you reproduce the issue in an older version?** Specify the most recent version where the issue does not occur.
- **Is the issue consistently reproducible?** If not, describe the frequency and conditions under which it happens.

**Configuration and environment details:** include the following information in your report:

- **Version of `adi_imu`** you're using.
- **Operating system name and version.**
- **Details about system-level dependencies and their compatibility.**

By providing this information, you’ll help us understand and address the issue more efficiently.

### Feature Requests

This guide will help you submit enhancement suggestions, including new features or improvements to existing functionality. Following these guidelines ensures maintainers and the community can understand your suggestion and identify related ideas.

#### Before Submitting an Enhancement Suggestion

- Ensure you're using the latest software version; newer versions may already include your desired feature.
- Check if an existing [library](https://index.ros.org/) provides the functionality you're requesting.

#### How to Submit a Good Enhancement Suggestion

Enhancement suggestions are tracked as GitHub issues. After determining the relevant repository, create an issue and include the following details:

1. **Use a clear and descriptive title** to summarize the suggestion.
2. **Describe the suggested enhancement** in detail, providing a step-by-step explanation of how it should work.
3. **Provide specific examples** to illustrate your suggestion, including code snippets as Markdown code blocks.
4. **Describe the current behavior** and explain the desired behavior, including why the change would be beneficial.
5. **Attach screenshots or GIFs** to visually explain your suggestion, if applicable.
6. **Justify why this enhancement would benefit most users** and explain why it shouldn’t be implemented as a separate application.
7. **Include version details**:
   - Version of the repository you're using.
   - Name and version of your operating system.

By following these steps, you’ll help ensure your suggestion is well-understood and can be considered effectively.


### Contributing documentation

Thank you for your interest in contributing to the project's documentation! For an in-depth understanding of the documentation system's design and structure, refer to the [Documentation Guidelines](./doc/user_guide/Documentation_Guidelines.rst)

#### Documentation Best Practices

We follow the documentation guidelines outlined in [Analog Devices Documentation Guidelines](https://analogdevicesinc.github.io/doctools/docs_guidelines.html). Please review these guidelines to ensure consistency and quality in your contributions.



## Becoming a Trusted Committers


Becoming a Trusted Committer is about consistently contributing value to the project and supporting its community. Here are some suggestions to help you grow into this role:

1. **Contribute Regularly**: Submit high-quality contributions, including code, documentation, and reviews, that align with the project's standards and best practices.

2. **Collaborate Actively**: Engage positively with maintainers and contributors by participating in discussions, offering constructive feedback, and fostering a collaborative environment.

3. **Follow Best Practices**: Adhere to the project's coding, documentation, and contribution guidelines to set a strong example for others.

4. **Be Responsive**: Actively review pull requests, respond to issues, and assist other contributors in a timely and respectful manner.

5. **Take Ownership**: Show initiative by taking responsibility for specific areas of the project, ensuring their quality, maintenance, and alignment with project goals.


## Licensing

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~


## Resources:
* [How to write a good commit message](https://cbea.ms/git-commit/) and [another resource](https://gist.github.com/rsp/057481db4dbd999bb7077f211f53f212).
* [Write better commits, build better projects](https://github.blog/2022-06-30-write-better-commits-build-better-projects/).
* [Good commit example (but extreme one)](https://dhwthompson.com/2019/my-favourite-git-commit).
* [How should a PR look like](https://opensource.com/article/18/6/anatomy-perfect-pull-request) and [anatomy of a PR](https://github.blog/2015-01-21-how-to-write-the-perfect-pull-request/).
* [Submitting patches](https://github.com/analogdevicesinc/linux/blob/master/Documentation/process/submitting-patches.rst).
