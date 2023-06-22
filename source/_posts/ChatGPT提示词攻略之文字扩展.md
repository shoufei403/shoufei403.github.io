---
title: ChatGPT提示词攻略(四)之文字扩展
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: 63bf453b
date: 2023-06-16 19:05:16
---

文字扩展是将较短的文本片段，例如一组指令或主题列表，交给大型语言模型生成更长的文本，例如基于已有的内容生成一封电子邮件或一篇关于某个主题的文章的任务。又或者你列出大纲，标题，让大模型填充对应的内容。



另外，还有一些很好的用途，例如将大型语言模型用作头脑风暴的伙伴。



但这里必须要提醒的是，ChatGPT生成的内容不一定准确，所以使用的时候要认真甄别。比如著名的stack overflow网站就曾被chatgpt的回答霸占，导致网站上充斥着不准确的回答。官方随即封禁了ChatGPT。

<!--more-->

在本文中，我们将通过一个示例演示如何使用语言模型基于某些信息生成个性化的电子邮件。这封电子邮件将明确说明来自一个AI机器人，以便提示用户这是自动生成的内容。



这个示例中，我们将撰写一封定制的电子邮件回复，以回应购买了搅拌机的客户评论。根据客户的评论和情感（正向还是负向），我们将生成一条定制的回复。现在我们将使用语言模型根据客户的评论和评论的情感生成一封定制的电子邮件。这里我们假设情感的正负向结果已经通过之前文章中提到的模型推理能力获取了。下面我们将根据客户评论的情感定制一封回复邮件。



```python
# given the sentiment from the lesson on "inferring",
# and the original customer message, customize the email
sentiment = "negative"

# review for a blender
chinese_review = f"""
所以，他们仍然在11月的季节性销售中以约49美元的价格销售17件套装，折扣约为一半，\
但由于某种原因（称其为价格欺诈），到了12月的第二周，同一套装的价格都上涨到了  \
70-89美元左右。11件套装的价格也比早期的29美元上涨了约10美元左右。所以看起来还 \
不错，但如果你看底座，刀片锁定的部分看起来不如几年前的早期版本那么好，但我打算  \
对它非常温柔（例如，我先在搅拌机中碾碎像豆子、冰、米等硬物，然后在搅拌机中将它  \
们粉碎到我想要的份量，然后切换到打蛋器刀片制作更细的面粉，制作冰沙时先使用交叉  \
切割刀片，然后如果需要更细/不那么浆状，再使用平刀片）。制作冰沙时的特别提示：  \
将要使用的水果和蔬菜切碎并冷冻（如果使用菠菜-轻轻炖软菠菜，然后冷冻，直到准备  \
使用-如果制作冰糕，使用小到中型食品加工机），这样你就可以避免添加太多冰块，如  \
果需要的话，可以在制作冰沙时添加。大约一年后，电机发出奇怪的声音。我打电话给客  \
户服务，但保修已经过期了，所以我不得不再买一个。FYI：这些产品的整体质量已经下  \
降了，所以他们有点依靠品牌认可和消费者忠诚度来维持销售。两天后收到了。
"""
```

```python
prompt = f"""
You are a customer service AI assistant.
Your task is to send an email reply to a valued customer.
Given the customer email delimited by ```, \
Generate a reply to thank the customer for their review.
If the sentiment is positive or neutral, thank them for \
their review.
If the sentiment is negative, apologize and suggest that \
they can reach out to customer service. 
Make sure to use specific details from the review.
Write in a concise and professional tone.
Sign the email as `AI customer agent`.
Response in chinese.
Customer review: ```{chinese_review}```
Review sentiment: {sentiment}
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
尊敬的客户，

感谢您对我们产品的评论。我们非常抱歉您对我们的产品不满意。我们深刻理解您的不满，并为此向您道歉。

我们建议您联系我们的客户服务团队，以便我们可以更好地了解您的问题，并为您提供更好的解决方案。我们将竭尽全力解决您的问题，以确保您对我们的服务感到满意。

再次感谢您的评论，我们将继续努力提高我们的产品和服务质量。

祝您一切顺利！

AI客户代理
```



## 理解temperature参数

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230615225003603.png)



语言模型的一个参数叫做“温度”，调节它将允许我们改变模型生成内容的多样性。你可以将“温度”看作是模型探索或随机性的程度。



对于图片中特定的短语，“我最喜欢的食物是”，模型预测的最有可能的下一个单词是“披萨”，其次是“寿司”和“玉米卷”。



在温度为零的情况下，模型将始终选择最有可能的下一个单词，即“披萨”，而在较高的温度值下，它也会选择其中一个不太可能的单词，甚至在更高的温度下，它可能会选择“玉米卷”，尽管玉米卷只有5%的可能性被选择。



一般来说，在构建需要可预测响应的应用程序时，我建议使用温度为零。比如基于确定的内容来回答问题。



如果你想生成更有创意的内容，你可能需要使用更高的温度值。比如NewBing上会有选项让你设置，以便生成符合你需求的内容。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230615235553163.png)



现在让我们使用更高的温度来生成一封电子邮件。这里我们设置一下“get_completion”函数中的温度参数，将其设置为0.7。



我们尝试三次，你可以看到每次生成的结果都不一样。

```python
尊敬的顾客，

感谢您对我们产品的评价。我们很抱歉您在使用过程中遇到了问题，我们一定会尽力改进我们的产品和服务。

我们深刻理解您对产品质量的关注，我们会把您的反馈转达给我们的研发团队。如果您需要更多的帮助，请联系我们的客户服务部门，他们将竭诚为您服务。

再次感谢您的评价，我们期待着您的下一次光临。

祝您生活愉快！

AI客户代理
```



```python
尊敬的客户，

非常感谢您对我们产品的评价。我们对您的不满深感抱歉。如果您对我们的产品有任何问题，请随时联系我们的客户服务部门以获取帮助。我们将竭尽所能，确保您得到最好的处理。

我们非常重视您对底座和刀片锁定的部分的反馈，并将确保将这些问题提供给我们的团队以进行改进。同时，我们很高兴听到您对我们产品的制作方式感到满意。

我们希望您会继续选择我们的产品，并感谢您的反馈。如果您有任何疑问或需要进一步帮助，请随时与我们联系。

祝您拥有美好的一天！

AI客户服务代表
```



```python
尊敬的顾客，

感谢您对我们的产品进行评论。我们非常抱歉您对我们的产品不满意。我们深刻理解您对我们产品质量的关注，我们将尽最大的努力来提高我们的服务质量。

由于您的经历，我们建议您联系我们的客户服务部门，以获得更好的支持和帮助。我们非常感谢您对我们的品牌的忠诚和信任，我们将不断努力提高我们的产品质量和服务水平。

祝您生活愉快。

AI客户代理
```



参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/7/expanding](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/7/expanding)





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)