---
title: 'Git Commit 的规范写法'
createTime: 2026/08/26 16:07:29
permalink: /notes/DevTools/git/commit-conventions.html
---

# Git Commit 的规范写法

## commit message 格式

```
<type>(<scope>): <subject>
类型(模块): 干了什么
```

### type（必须）

用于说明git commit的类别，常见的有下面的标识。

| type               | 含义                            | 示例                                        |
| ------------------ | ------------------------------- | ------------------------------------------- |
| `feat（常用）`     | 新增功能                        | `feat(user): add profile page`              |
| `fix（常用）`      | 修复 bug                        | `fix(login): handle invalid password`       |
| `docs（常用）`     | 修改文档                        | `docs: update README`                       |
| `style`            | 代码格式调整，不影响逻辑        | `style: format code`                        |
| `refactor（常用）` | 重构代码，但不是新增功能/修 bug | `refactor(auth): simplify token validation` |
| `perf`             | 性能优化                        | `perf(db): reduce query time`               |
| `test`             | 添加或修改测试                  | `test(user): add login tests`               |
| `build`            | 构建系统、依赖相关              | `build: update vite version`                |
| `ci`               | CI/CD 配置相关                  | `ci: add GitHub Actions workflow`           |
| `chore（常用）`    | 杂项维护                        | `chore: update gitignore`                   |
| `revert`           | 回滚之前的提交                  | `revert: revert login changes`              |

### scope（可选）

scope用于说明 commit 影响的范围，比如数据层、控制层、[视图层](https://zhida.zhihu.com/search?content_id=128701224&content_type=Article&match_order=1&q=视图层&zd_token=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJ6aGlkYV9zZXJ2ZXIiLCJleHAiOjE3ODc4OTk2OTIsInEiOiLop4blm77lsYIiLCJ6aGlkYV9zb3VyY2UiOiJlbnRpdHkiLCJjb250ZW50X2lkIjoxMjg3MDEyMjQsImNvbnRlbnRfdHlwZSI6IkFydGljbGUiLCJtYXRjaF9vcmRlciI6MSwiemRfdG9rZW4iOm51bGx9.mdNSJJ9xMOv-mcEfGtUTyHTiMsutseIIonYrsU5EctU&zhida_source=entity)等等，视项目不同而不同。感觉一般的个人小项目，不写也没事。

### subject（必须）

subject是commit目的的简短描述（中英皆可），不超过50个字符，结尾不加句号或其他标点符号。
