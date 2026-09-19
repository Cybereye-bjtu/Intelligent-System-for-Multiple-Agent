# 仓库发布流程

## 范围

本仓库同时保存服务器端多车系统、两端共享接口，以及 205 型车辆的边缘运动控制。
生成目录、运行日志、地图采集物和真实 MQTT/SSH 凭据不得进入 Git。

## 分支与审查

```bash
git switch -c feat/205-edge-deployment
git status --short
git diff --check
git diff --cached
git commit -m "feat: package validated 205 edge navigation deployment"
git push -u <remote> feat/205-edge-deployment
```

通过 Pull Request 合入团队 `main`。合入前必须完成：

1. Python 单元测试与 ROS 2 包构建；
2. 凭据扫描；
3. 205 静态部署验证；
4. Gateway 默认关闭确认；
5. 一台车低速回归后再为提交打版本标签。

建议首个团队发布标签为 `v1.1.0-edge-205`。团队成员应部署固定标签，而不是任意
时间点的 `main`。

## GitHub远程

服务器历史工作树的 `origin` 指向个人仓库。建议保留个人仓库为 `fork`，团队仓库
配置为 `upstream`：

```bash
git remote rename origin fork
git remote add upstream git@github.com:Cybereye-bjtu/Intelligent-System-for-Multiple-Agent.git
git fetch upstream
```

服务器当前需要先配置有团队仓库权限的 GitHub SSH 密钥，或使用 `gh auth login`
完成授权。不得把访问令牌写入远程 URL、脚本或仓库文件。
