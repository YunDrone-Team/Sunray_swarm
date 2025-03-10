<img src="https://pic1.imgdb.cn/item/67ceabf3066befcec6e26c76.png" alt="yundrone logo" align="right" height="100" />

## Sunray_swarm - 多智能体集群控制开源项目

Sunray_swarm是室内多智能体集群编队实验平台“智群·RMTT”的配套开源代码，接入了RMTT无人机和麦克纳姆轮无人车的驱动，提供了单体控制、集群控制及路径规划等相关功能demo，支持开发者在ROS环境中对集群无人机/无人车进行算法二次开发。本项目支持RVIZ仿真及真实平台实验，且提供完善的二次开发手册。

### “智群·RMTT”平台介绍

<div style="text-align:center">
  <img src="https://resource-wangsu.helplook.net/docker_production/5n3bi9/article/fBjfd8tA/678f41c58b3b2.png" alt="Markdown Logo" style="width:400px; height:auto;" />
</div>

“智群·RMTT”是一套基于无人机和无人车的室内多智能体集群编队实验平台，配套丰富的二次开发例程和简洁的集群控制地面站，支持集群控制、路径规划、目标识别、任务决策等教学任务和算法验证。实验平台基于高精度光学动作捕捉系统，能够在室内实时获得多智能体的三维空间位姿，实现了多智能体的闭环稳定控制。小型化的无人机和无人车平台安全可靠且基于ROS可互通互联，适合进行集群类算法的教学和开发验证。集群控制地面站可同时接入所有智能体，能够实时读取位置、速度和视频流等状态，支持单体/集群控制指令的下达。此外，实验平台配套了单体/集群控制、路径规划等算法案例及文档，且提供线下交付培训及线上长期答疑服务，助力用户的快速上手使用及二次开发。
- **应用对象**：主要针对高校师生、科研人员、工程技术人员及机器人爱好者，适合用于机器人学、无人系统、智能控制、人工智能等专业的实验、研究、开发与验证工作。
- **视频介绍**：[“智群·RMTT”产品视频](https://www.bilibili.com/video/BV1sw6mYEEJy/?share_source=copy_web&vd_source=0fc5f616d655707c69c3292e4afd541e)


### 快速入门
 - 安装及使用：[“智群·RMTT”使用文档](https://wiki.yundrone.cn/catalog/rmtt_doc)
    - 需掌握ROS编程基础。
    - 需掌握C语言基础（大部分程序为C语言，部分模块有少量C++和python的语法）。
    
### 联系我们
广州云纵科技有限公司是一家专业从事无人机软硬件和解决方案定制的公司。项目合作、无人机软硬件、实验室整体解决方案定制，请添加微信“19120231228”（备注消息：智群RMTT）。
- 公司地址：[广州市南沙区悦方中心1203房](https://map.baidu.com/poi/%E5%B9%BF%E5%B7%9E%E4%BA%91%E7%BA%B5%E7%A7%91%E6%8A%80%E6%9C%89%E9%99%90%E5%85%AC%E5%8F%B8/@12635819.79948515,2593492.005733868,19z?uid=32c1e91366ffdbb7ad045788&ugc_type=3&ugc_ver=1&device_ratio=1&compat=1&pcevaname=pc4.1&querytype=detailConInfo&da_src=shareurl)

<img src="https://cdn.yun.sooce.cn/6/53163/png/172973873083794134484fbe345a37638dc620eaf6e50.png?imageMogr2/thumbnail/80x&version=0" alt="yundrone logo" align="right" height="150" />

- 相关链接：
  - [云纵科技官网](http://www.yundrone.cn/index.html)
  - [云纵科技产品文档](https://wiki.yundrone.cn/)
  - [云纵科技淘宝店](https://5q239j0txjkacow9mk5tofi9dvxs6st.taobao.com/index.htm?spm=a1z10.1-c-s.w5002-25336597030.2.5c854fd4rOYblf)
  - [云纵科技Bilibili](https://space.bilibili.com/3546736714844413)
  - 小红书：打开小红书搜索“云纵科技”
  - 微信公众号：打开微信搜索“云纵科技”或扫描右方二维码




### 致谢
- 感谢[北京度量科技有限公司](https://www.nokov.com/)提供的动作捕捉设备进行产品开发测试
- 感谢西安天之博特有限公司开源的[rmtt_ros功能包](https://github.com/tianbot/rmtt_ros)
- 感谢北卡罗来纳大学教堂山分校Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, and Dinesh Manocha开源的[RVO算法库](https://gamma.cs.unc.edu/RVO2/)

### 版权声明

 - 本开源项目仅限个人使用，请勿用于商业用途。
 - 如利用本项目进行营利活动，云纵科技将追究侵权行为。