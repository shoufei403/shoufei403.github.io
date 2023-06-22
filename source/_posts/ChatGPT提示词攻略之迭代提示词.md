---
title: ChatGPT提示词攻略(二)之迭代提示词
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: 5fe3eac2
date: 2023-06-15 09:03:16
---

当我们在调试程序时，通常很难一次就把程序正常跑起来。这是普遍现象。但我们会借助一些工具和手段，有步骤有流程地去调整程序，最终让程序按照我们想要的样子正常执行。



对于提示词来说也是一样的。当我们向ChatGPT提问时，一开始它给我们的答案可能并不理想。但是当我们逐步去调整提示词，慢慢地，它的回答就会接近我们想要的答案。



所以这里最重要的是我们如何去迭代提示词？

<!--more-->

## 迭代提示词的框架

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230607214947600.png)

这个过程和我们调试程序的过程基本是一致的。先写初版的提示词，然后给到ChatGPT，试着让它回答一下。得到回答后，分析回答的结果，找出不足的地方，修改提示词。这样一步一步迭代。



这里也要注意一下，就是在写提示词时记住之前提高过的《书写提示词基本原则》。

- 提示词的书写要清晰，带有明确的指令
- 给模型时间去思考，即指明模型的思考过程



## 例子



```python
chinese_fact_sheet_chair = """

产品概述：
- 属于美丽的中世纪风格办公家具系列，包括文件柜、书桌、书架、会议桌等。
- 有多种外壳颜色和底座饰面可选。
- 可以选择塑料背面和前面的软垫（SWC-100）或10种织物和6种皮革的全套软垫（SWC-110）。
- 底座饰面选项有：不锈钢、哑光黑色、光泽白色或铬。
- 椅子可以选择有或没有扶手。
- 适用于家庭或商业环境。
- 合格的合同使用。

结构：
- 5个轮子的塑料涂层铝底座。
- 气动椅子调节，方便升降操作。

尺寸：
- 宽度53厘米|20.87英寸
- 深度51厘米|20.08英寸
- 高度80厘米|31.50英寸
- 座高44厘米|17.32英寸
- 座深41厘米|16.14英寸

选项：
- 软地板或硬地板脚轮选项。
- 两种座椅泡沫密度可选：中等（1.8磅/立方英尺）或高（2.8磅/立方英尺）
- 无扶手或8个位置的PU扶手

材料：
外壳底座滑轮
- 铸铝，改性尼龙PA6 / PA66涂层。
- 外壳厚度：10毫米。
座位
- HD36泡沫

原产国：
- 意大利

"""
```

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information
provided in the technical specifications delimited by 
triple backticks.


Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
产品描述：

这款美丽的中世纪风格办公家具系列包括文件柜、书桌、书架、会议桌等，适用于家庭或商业环境。您可以选择多种外壳颜色和底座饰面，以及塑料背面和前面的软垫或10种织物和6种皮革的全套软垫。底座饰面选项有不锈钢、哑光黑色、光泽白色或铬。椅子可以选择有或没有扶手。此外，我们还提供软地板或硬地板脚轮选项，两种座椅泡沫密度可选，以及无扶手或8个位置的PU扶手。

这款椅子的结构采用5个轮子的塑料涂层铝底座，气动椅子调节，方便升降操作。尺寸为宽度53厘米，深度51厘米，高度80厘米，座高44厘米，座深41厘米。外壳底座滑轮采用铸铝，改性尼龙PA6 / PA66涂层，外壳厚度为10毫米。座位采用HD36泡沫，质量可靠。

这款椅子的原产国是意大利，是一款合格的合同使用产品。无论您是在家中还是在办公室使用，这款椅子都能为您提供舒适的座椅体验。
```

这里根据提供的产品材料写一个对产品的描述。

如果觉得描述有些长，我们可以添加限定字数或者语句数量的方式来限制回复的长度。

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information
provided in the technical specifications delimited by 
triple backticks.

回答要求在100字以内。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
这款中世纪风格的办公家具系列包括文件柜、书桌、书架、会议桌等，适用于家庭或商业环境。可选多种外壳颜色和底座饰面，底座饰面选项有不锈钢、哑光黑色、光泽白色或铬。椅子可以选择有或没有扶手，座位可选中等或高密度泡沫。座椅采用气动调节，方便升降操作。座位宽度53厘米，深度51厘米，高度80厘米，座高44厘米，座深41厘米。外壳底座滑轮采用铸铝，改性尼龙PA6 / PA66涂层，外壳厚度为10毫米。原产国为意大利。
```

看起来缩短了很多。但具体字数不一定那么精确到100字。因为模型内部是用token计数的，有时一个字会分成多个token。另外，它算数也不咋地。



```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information
provided in the technical specifications delimited by 
triple backticks.

要求使用三句话来回答。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)

```

回答：

```python

这款中世纪风格的办公家具系列包括文件柜、书桌、书架、会议桌等，适用于家庭或商业环境。可选多种外壳颜色和底座饰面，底座饰面选项有不锈钢、哑光黑色、光泽白色或铬。 

椅子采用5个轮子的塑料涂层铝底座，气动椅子调节，方便升降操作。座位采用HD36泡沫，可选两种座椅泡沫密度：中等或高。可选择有或没有扶手，座椅还有软地板或硬地板脚轮选项。
```

不管有没有精确到三句话。通过添加限定语句数量的提示词，模型的回答是靠近我们的需求的。



如果我们希望模型的回答更具有技术性，并且回答要突出制造产品的材料。

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information 
provided in the technical specifications delimited by 
triple backticks.

描述是针对家具零售商,所以应该是技术性的，并把重点放在
制造产品的材料。

要求使用三句话来回答。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
这款美丽的中世纪风格办公家具系列包括文件柜、书桌、书架、会议桌等，适用于家庭或商业环境。外壳底座滑轮采用铸铝和改性尼龙PA6 / PA66涂层，座位采用HD36泡沫，可选择软地板或硬地板脚轮选项，座椅泡沫密度可选中等或高，无扶手或8个位置的PU扶手。这款家具系列是意大利制造，符合合格的合同使用。
```



如果希望把技术描述中的产品ID提取出来。

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information 
provided in the technical specifications delimited by 
triple backticks.

在描述的最后放置技术描述中提到的所有产品ID。

描述是针对家具零售商,所以应该是技术性的，并把重点放在
制造产品的材料。

要求使用三句话来回答。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
这款中世纪风格的办公家具系列包括文件柜、书桌、书架和会议桌等，适用于家庭或商业环境。外壳底座滑轮采用铸铝和改性尼龙PA6 / PA66涂层，座位采用HD36泡沫。产品尺寸为宽53厘米、深51厘米、高80厘米，座高44厘米，座深41厘米。产品ID：SWC-100和SWC-110。
```



要求把技术描述中的尺寸信息提取出来，形成一个表格。

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information 
provided in the technical specifications delimited by 
triple backticks.

在描述的最后放置技术描述中提到的所有产品ID。

描述是针对家具零售商,所以应该是技术性的，并把重点放在
制造产品的材料。

在描述之后，添加一个表，该表给出产品的尺寸。该表应该有两列。
在第一列中包含维度的名称。在第二列中只包括英寸的测量值。

表的抬头设置为“产品尺寸”。

要求使用三句话来回答。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python

这款中世纪风格的办公家具系列包括文件柜、书桌、书架和会议桌等，适用于家庭或商业环境。您可以选择多种外壳颜色和底座饰面，以及软垫和扶手的选项。座位采用高密度泡沫，外壳底座滑轮采用铸铝和改性尼龙PA6 / PA66涂层。产品尺寸为20.87英寸宽，20.08英寸深，31.50英寸高，座高17.32英寸，座深16.14英寸。请注意产品ID。 

| 产品尺寸 | 英寸 |
| -------- | ---- |
| 宽度     | 20.87 |
| 深度     | 20.08 |
| 高度     | 31.50 |
| 座高     | 17.32 |
| 座深     | 16.14 | 

产品ID：SWC-100, SWC-110.
```



最后我们要求输出的内容是HTML格式的。

```python
prompt = f"""
Your task is to help a marketing team create a 
description for a retail website of a product based 
on a technical fact sheet.

Write a chinese product description based on the information 
provided in the technical specifications delimited by 
triple backticks.

在描述的最后放置技术描述中提到的所有产品ID。

描述是针对家具零售商,所以应该是技术性的，并把重点放在
制造产品的材料。

在描述之后，添加一个表，该表给出产品的尺寸。该表应该有两列。
在第一列中包含维度的名称。在第二列中只包括英寸的测量值。

表的抬头设置为“产品尺寸”。

要求使用三句话来回答。

使用HTML输出所有内容，并把生成的描述放置在<div>元素中。

Technical specifications: ```{chinese_fact_sheet_chair}```
"""
response = get_completion(prompt)
print(response)
```

回答效果如下：

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230607212629112.png)



**经过测试发现**，当我把提示词换成中文后，给出回答的反应时间变长了，回复的准确度也有所下降。



参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/3/iterative](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/3/iterative)



---

**觉得有用就点个赞吧！**

我是首飞，做有趣的事情，拿出来分享。









