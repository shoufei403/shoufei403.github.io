---
title: ChatGPT提示词攻略(五)之推理
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: eae62d56
date: 2023-06-17 18:15:16
---

这节介绍大模型判断文字的语义，或者说对内容进行情感分析的能力。同时也演示了大模型如何提取出文字中的关键信息。



在传统的机器学习方案中，要做到对文字内容的情感分析，需要先对一系列的文字内容（如评论）进行人工标注。把这些文字内容人工分类成“正向”和负向“，然后再喂给一个机器学习模型去训练，得到一组参数。模型训练好后再部署好，把需要判断的未标注文字内容给到训练好的模型，让它判断一下文字内容的情感倾向。



可以看到，对于传统的机器学习方案，有很多工作需要做。而且这个训练出来的模型也只能干这一件事情。如果我们想要提取文字内容中的关键信息又得重新训练另外一个单独的模型。

<!--more-->

在大模型时代，做到这些我们仅仅只要写一下提示词。



下面是一个购买台灯客户的评论。这个例子演示了如何使用大模型来分析文字内容的情绪。

```python
lamp_review = """
Needed a nice lamp for my bedroom, and this one had \
additional storage and not too high of a price point. \
Got it fast.  The string to our lamp broke during the \
transit and the company happily sent over a new one. \
Came within a few days as well. It was easy to put \
together.  I had a missing part, so I contacted their \
support and they very quickly got me the missing piece! \
Lumina seems to me to be a great company that cares \
about their customers and products!!
"""
```

```python
prompt = f"""
What is the sentiment of the following product review, 
which is delimited with triple backticks?

Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
The sentiment of the product review is positive.
```

```python
prompt = f"""
What is the sentiment of the following product review, 
which is delimited with triple backticks?

Give your answer as a single word, either "positive" \
or "negative".

Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
positive
```



```python
lamp_deliver_review = """
快递太慢了。很久都没有收到货。
"""
```

```python
prompt = f"""
What is the sentiment of the following product review, 
which is delimited with triple backticks?

Give your answer as a single word, either "正向" \
or "负向".

Review text: '''{lamp_deliver_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
负向
```



### 分辨出情绪的类型

```python
prompt = f"""
Identify a list of emotions that the writer of the \
following review is expressing. Include no more than \
five items in the list. Format your answer as a list of \
lower-case words separated by commas.

Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
happy, satisfied, grateful, impressed, content
```



### 判断是否生气

```python
prompt = f"""
Is the writer of the following review expressing anger?\
The review is delimited with triple backticks. \
Give your answer as either yes or no.

Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
No
```



### 从评论中提取产品和公司信息

```python
prompt = f"""
Identify the following items from the review text: 
- Item purchased by reviewer
- Company that made the item

The review is delimited with triple backticks. \
Format your response as a JSON object with \
"Item" and "Brand" as the keys. 
If the information isn't present, use "unknown" \
as the value.
Make your response as short as possible.
  
Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
{
  "Item": "lamp",
  "Brand": "Lumina"
}
```



### 多任务执行

这里演示了同时进行情感分析和提取关键信息。

```python
prompt = f"""
Identify the following items from the review text: 
- Sentiment (positive or negative)
- Is the reviewer expressing anger? (true or false)
- Item purchased by reviewer
- Company that made the item

The review is delimited with triple backticks. \
Format your response as a JSON object with \
"Sentiment", "Anger", "Item" and "Brand" as the keys.
If the information isn't present, use "unknown" \
as the value.
Make your response as short as possible.
Format the Anger value as a boolean.

Review text: '''{lamp_review}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
{
  "Sentiment": "positive",
  "Anger": false,
  "Item": "lamp with additional storage",
  "Brand": "Lumina"
}
```



### 推理

分析一段文字中讲述了哪些话题。

```python
story = """
In a recent survey conducted by the government, 
public sector employees were asked to rate their level 
of satisfaction with the department they work at. 
The results revealed that NASA was the most popular 
department with a satisfaction rating of 95%.

One NASA employee, John Smith, commented on the findings, 
stating, "I'm not surprised that NASA came out on top. 
It's a great place to work with amazing people and 
incredible opportunities. I'm proud to be a part of 
such an innovative organization."

The results were also welcomed by NASA's management team, 
with Director Tom Johnson stating, "We are thrilled to 
hear that our employees are satisfied with their work at NASA. 
We have a talented and dedicated team who work tirelessly 
to achieve our goals, and it's fantastic to see that their 
hard work is paying off."

The survey also revealed that the 
Social Security Administration had the lowest satisfaction 
rating, with only 45% of employees indicating they were 
satisfied with their job. The government has pledged to 
address the concerns raised by employees in the survey and 
work towards improving job satisfaction across all departments.
"""
```



```python
prompt = f"""
Determine five topics that are being discussed in the \
following text, which is delimited by triple backticks.

Make each item one or two words long. 

Format your response as a list of items separated by commas.

Text sample: '''{story}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
government survey, job satisfaction, NASA, Social Security Administration, employee concerns
```



判断文字内容中是否有讲述相关话题内容。

```python
topic_list = [
    "nasa", "local government", "engineering", 
    "employee satisfaction", "federal government"
]
```

```python
prompt = f"""
Determine whether each item in the following list of \
topics is a topic in the text below, which
is delimited with triple backticks.

Give your answer as list with 0 or 1 for each topic.\

List of topics: {", ".join(topic_list)}

Text sample: '''{story}'''
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
nasa: 1
local government: 0
engineering: 0
employee satisfaction: 1
federal government: 1
```

判断是否有NASA相关内容。

```python
topic_dict = {i.split(': ')[0]: int(i.split(': ')[1]) for i in response.split(sep='\n')}
if topic_dict['nasa'] == 1:
    print("ALERT: New NASA story!")
```

输出：

```python
ALERT: New NASA story!
```





参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/5/inferring](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/5/inferring)



---

**觉得有用就点个赞吧！**

我是首飞，做有趣的事情，拿出来分享。