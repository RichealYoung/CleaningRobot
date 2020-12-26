# 环境依赖
matplotlib==3.3.2\
numpy==1.19\
pandas==1.1.3\
gekko==0.2.8
# 目录结构
```python
│ ─helper.py                        #辅助函数
│ ─Path_plan.py                     #主函数
│ ─Readme.md                        #说明文档
│ ─requirements.txt                 #环境依赖
│ ─Final_project.pdf                #报告文件
│
└─model_$System time$               #输出目录
      ─Best Path Plan.png           #最佳路径示意图
      ─Best Path Plan.json          #最佳路径x,y,theta详细数据
```
# 使用方法
0. 部署环境
```shell
pip install -r ./requirements.txt
```
1. 运行Path_plan.py脚本
2. 标准输出如下所示
```shell
Time consumes:2.798s
Best Path Plan.png is saved in ../model_$System time$
Best Path Plan.json is saved in ../model_$System time$
We use 57 steps with distacne:24.86779340321863
```
3. Path_plan.py脚本的同级目录下会出现文件夹model_\$System time\$，其中包含Best Path Plan.png与Best Path Plan.json
4. Best Path Plan.png为最佳路径示意图
5. Best Path Plan.json为最佳路径x,y,theta的详细数据
