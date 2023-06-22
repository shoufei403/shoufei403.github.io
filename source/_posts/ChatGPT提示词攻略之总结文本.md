---
title: ChatGPT提示词攻略(六)之总结文本
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: 4122fbc2
date: 2023-06-18 10:03:16
---


现在是信息爆炸时代，打开手机，各种文章扑面而来。我们的精力是有限的。如果有人帮忙把文章总结好给我们，这不就节省了很多时间嘛！我们也就可以阅读更多的文章了。



恰好大语言模型在总结文章方面非常有天赋。

<!--more-->

下面来看看示例。

```python
prod_review = """
Got this panda plush toy for my daughter's birthday, \
who loves it and takes it everywhere. It's soft and \ 
super cute, and its face has a friendly look. It's \ 
a bit small for what I paid though. I think there \ 
might be other options that are bigger for the \ 
same price. It arrived a day earlier than expected, \ 
so I got to play with it myself before I gave it \ 
to her.
"""

prompt = f"""
Your task is to generate a short summary of a product \
review from an ecommerce site. 

Summarize the review below, delimited by triple 
backticks, in at most 30 words. 

Review: ```{prod_review}```
"""

response = get_completion(prompt)
print(response)

```

回答：

```python

Soft and cute panda plush toy loved by daughter, but a bit small for the price. Arrived early.
```



如果总结的时候，我们更关注快递方面的信息呢？

```python
prompt = f"""
Your task is to generate a short summary of a product \
review from an ecommerce site to give feedback to the \
Shipping deparmtment. 

Summarize the review below, delimited by triple 
backticks, in at most 30 words, and focusing on any aspects \
that mention shipping and delivery of the product. 

Review: ```{prod_review}```
"""

response = get_completion(prompt)
print(response)
```

回答：

```python
The panda plush toy arrived a day earlier than expected, but the customer felt it was a bit small for the price paid.
```

很明显，这里特地提到了The panda plush toy比期望的时间早一天到达。



如果总结的时候，我们更关注价格方面的信息呢？

```python
prompt = f"""
Your task is to generate a short summary of a product \
review from an ecommerce site to give feedback to the \
pricing deparmtment, responsible for determining the \
price of the product.  

Summarize the review below, delimited by triple 
backticks, in at most 30 words, and focusing on any aspects \
that are relevant to the price and perceived value. 

Review: ```{prod_review}```
"""

response = get_completion(prompt)
print(response)
```

回答：

```python
The panda plush toy is soft, cute, and loved by the recipient, but the price may be too high for its size.
```

总结文字中提到了The panda plush toy不值这个价，因为尺寸对不起这么高的价格。



另外我们也可以从文本中抽取需要的关键信息。

```python
prompt = f"""
Your task is to extract relevant information from \ 
a product review from an ecommerce site to give \
feedback to the Shipping department. 

From the review below, delimited by triple quotes \
extract the information relevant to shipping and \ 
delivery. Limit to 30 words. 

Review: ```{prod_review}```
"""

response = get_completion(prompt)
print(response)
```

回答：

```python
The product arrived a day earlier than expected.
```

这里提取了物流方面的信息。



总结一下，大语言模型可以做：

- 文章总结，并且可以设置总结的侧重点
- 提取文章中的关键信息



参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/4/summarizing](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/4/summarizing)





---

**觉得有用就点个赞吧！**

我是首飞，做有趣的事情，拿出来分享。