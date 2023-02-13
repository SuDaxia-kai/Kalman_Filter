# README

Kalman滤波二位实例的C++实现。数据可视化通过文件流输出并使用EXCEL画图。

## Getting Started

此项目在标准库的基础上再多装个Eigen库就可正常运行。本项目的Eigen版本为3.4.0

### Installation

Eigen官网：http://eigen.tuxfamily.org/index.php?title=Main_Page#Download

安装完毕后随便找个地方解压

在`c_cpp_properties.json`中的`include path`中添加路径

```json
"includePath": [
             "${workspaceFolder}\\**",
				//...
              "D:/eigen3/eigen3/**"
          ],
```

在`tasks,json`中的参数`args`中添加执行命令

```json
"args": [
			// ....
            "-I",
            "D:/eigen3/eigen3"
        ],
```

## release History

- 1.0.0
  - 实现Kalman Filter，但未经过数据检测与验证
- 1.0.1
  - 用文件流输出并用EXCEL进行数据验证

## Author

- **Daxia Su** - a student of GDUT

## Results

<img src=".\Fig1position.png" style="zoom: 67%;" />

<img src=".\Fig2velocity.png" style="zoom: 67%;" />
