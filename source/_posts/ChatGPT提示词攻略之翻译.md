---
title: ChatGPT提示词攻略(三)之翻译
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: a65f6821
date: 2023-06-16 08:13:16
---

在本篇文章中，我们将探讨如何使用大语言模型进行文本转换任务，例如语言翻译、拼写和语法检查、语气调整和格式转换。



## 翻译

ChatGPT接受多种语言的训练，使得模型具备翻译能力。以下是如何使用这种能力的一些示例。

```python
prompt = f"""
Translate the following English text to chinese: \ 
​```Hi, I would like to order a blender```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
你好，我想订购一个搅拌机。
```
<!--more-->


识别句子使用了何种语言。

```python
prompt = f"""
Tell me which language this is: 
​```你好，我叫首飞```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
This is Chinese (Mandarin).
```



这里演示了同时将一句话转换为多种语言。

```python
prompt = f"""
Translate the following  text to Chinese and Japanese
and Korean: \
​```I want to order a basketball```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
Chinese: 我想订购一个篮球 (Wǒ xiǎng dìnggòu yīgè lánqiú)
Japanese: バスケットボールを注文したいです (Basuketto bōru o chūmon shitai desu)
Korean: 저는 농구공을 주문하고 싶어요 (Jeoneun nonggugong-eul jumunhago sip-eoyo)
```



下面的例子演示，大模型改变句子的语气，是正式还是不正式？

```python
prompt = f"""
Translate the following text to Chinese in both the \
formal and informal forms: 
'Would you like to order a pillow?'
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
Formal: 您需要订购枕头吗？
Informal: 你想订购枕头吗？
```



## 语气转换

写作可以根据预期受众而有所不同。ChatGPT可以产生不同的语气。

```python
prompt = f"""
Translate the following from slang to a business letter: 
'Dude, This is Joe, check out this spec on this standing lamp.'
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
Dear Sir/Madam,

I am writing to bring to your attention a standing lamp that I believe may be of interest to you. Please find attached the specifications for your review.

Thank you for your time and consideration.

Sincerely,

Joe
```



## 格式转换

ChatGPT可以在格式之间进行转换。只要提示词描述好输入和输出格式。

```python
data_json = { "resturant employees" :[ 
    {"name":"Shyam", "email":"shyamjaiswal@gmail.com"},
    {"name":"Bob", "email":"bob32@gmail.com"},
    {"name":"Jai", "email":"jai87@gmail.com"}
]}

prompt = f"""
Translate the following python dictionary from JSON to an HTML \
table with column headers and title: {data_json}
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
<table>
  <caption>Restaurant Employees</caption>
  <thead>
    <tr>
      <th>Name</th>
      <th>Email</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Shyam</td>
      <td>shyamjaiswal@gmail.com</td>
    </tr>
    <tr>
      <td>Bob</td>
      <td>bob32@gmail.com</td>
    </tr>
    <tr>
      <td>Jai</td>
      <td>jai87@gmail.com</td>
    </tr>
  </tbody>
</table>
```

```python
from IPython.display import display, Markdown, Latex, HTML, JSON
display(HTML(response))
```

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230614085019525.png)



## 拼写和语法检查

以下是一些常见的语法和拼写问题以及LLM的回应。

```python
text = [ 
  "The girl with the black and white puppies have a ball.",  # The girl has a ball.
  "Yolanda has her notebook.", # ok
  "Its going to be a long day. Does the car need it’s oil changed?",  # Homonyms
  "Their goes my freedom. There going to bring they’re suitcases.",  # Homonyms
  "Your going to need you’re notebook.",  # Homonyms
  "That medicine effects my ability to sleep. Have you heard of the butterfly affect?", # Homonyms
  "This phrase is to cherck chatGPT for speling abilitty"  # spelling
]
for t in text:
    prompt = f"""Proofread and correct the following text
    and rewrite the corrected version. If you don't find
    and errors, just say "No errors found". Don't use 
    any punctuation around the text:
    ```{t}```"""
    response = get_completion(prompt)
    print(response)
```

回答：

```python
The girl with the black and white puppies has a ball.
No errors found.
It's going to be a long day. Does the car need its oil changed?
Their goes my freedom. There going to bring they're suitcases.

Corrected version: 
There goes my freedom. They're going to bring their suitcases.
You're going to need your notebook.
That medicine affects my ability to sleep. Have you heard of the butterfly effect?
This phrase is to check ChatGPT for spelling ability.
```



```python
text = f"""
Got this for my daughter for her birthday cuz she keeps taking \
mine from my room.  Yes, adults also like pandas too.  She takes \
it everywhere with her, and it's super soft and cute.  One of the \
ears is a bit lower than the other, and I don't think that was \
designed to be asymmetrical. It's a bit small for what I paid for it \
though. I think there might be other options that are bigger for \
the same price.  It arrived a day earlier than expected, so I got \
to play with it myself before I gave it to my daughter.
"""
prompt = f"proofread and correct this review: ```{text}```"
response = get_completion(prompt)
print(response)
```

回答：

```python
I got this for my daughter's birthday because she keeps taking mine from my room. Yes, adults also like pandas too. She takes it everywhere with her, and it's super soft and cute. However, one of the ears is a bit lower than the other, and I don't think that was designed to be asymmetrical. Additionally, it's a bit small for what I paid for it. I think there might be other options that are bigger for the same price. On the positive side, it arrived a day earlier than expected, so I got to play with it myself before I gave it to my daughter.
```

显示差异

```python
from redlines import Redlines

diff = Redlines(text,response)
display(Markdown(diff.output_markdown))
```

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230614090200541.png)



```python
prompt = f"""
proofread and correct this review. Make it more compelling. 
Ensure it follows APA style guide and targets an advanced reader. 
Output in markdown format.
Text: ```{text}```
"""
response = get_completion(prompt)
display(Markdown(response))
```

**注意**，APA style是一种用于学术写作和出版的规范和指南，它由美国心理学会（American Psychological Association）制定和发布。APA style提供了一套标准的格式和风格要求，以确保学术写作的清晰性、准确性、一致性和规范性。它涵盖了学术论文的各个方面，包括标题页、摘要、正文、参考文献等，并规定了引用和参考文献的格式和要求。

APA style通常用于社会科学领域的学术论文和出版物，如心理学、教育学、社会学、护理学、商务管理等。在美国和其他英语国家，APA style已成为学术写作的标准要求之一。

回答：

```python
Title: A Soft and Cute Panda Plush Toy for All Ages

Introduction: As a parent, finding the perfect gift for your child's birthday can be a daunting task. However, I stumbled upon a soft and cute panda plush toy that not only made my daughter happy but also brought joy to me as an adult. In this review, I will share my experience with this product and provide an honest assessment of its features.

Product Description: The panda plush toy is made of high-quality materials that make it super soft and cuddly. Its cute design is perfect for children and adults alike, making it a versatile gift option. The toy is small enough to carry around, making it an ideal companion for your child on their adventures.

Pros: The panda plush toy is incredibly soft and cute, making it an excellent gift for children and adults. Its small size makes it easy to carry around, and its design is perfect for snuggling. The toy arrived a day earlier than expected, which was a pleasant surprise.

Cons: One of the ears is a bit lower than the other, which makes the toy asymmetrical. Additionally, the toy is a bit small for its price, and there might be other options that are bigger for the same price.

Conclusion: Overall, the panda plush toy is an excellent gift option for children and adults who love cute and cuddly toys. Despite its small size and asymmetrical design, the toy's softness and cuteness make up for its shortcomings. I highly recommend this product to anyone looking for a versatile and adorable gift option.
```





参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/6/transforming](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/6/transforming)





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

