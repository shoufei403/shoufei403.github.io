---
title: 如何使用ChatGPT的API(六)输出检测
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: 93a461f2
date: 2023-09-30 20:13:16
---


本篇文章介绍如何检测模型的输出。



之前我们介绍过如何通过Moderation接口检测用户的输入，以避免让模型接受到违规的或者不合适的输入。现在我们同样也可以使用Moderation接口来检测模型输出的内容。



## 检测输出内容是否有害

```python
final_response_to_customer = f"""
The SmartX ProPhone has a 6.1-inch display, 128GB storage, \
12MP dual camera, and 5G. The FotoSnap DSLR Camera \
has a 24.2MP sensor, 1080p video, 3-inch LCD, and \
interchangeable lenses. We have a variety of TVs, including \
the CineView 4K TV with a 55-inch display, 4K resolution, \
HDR, and smart TV features. We also have the SoundMax \
Home Theater system with 5.1 channel, 1000W output, wireless \
subwoofer, and Bluetooth. Do you have any specific questions \
about these products or any other products we offer?
"""
response = openai.Moderation.create(
    input=final_response_to_customer
)
moderation_output = response["results"][0]
print(moderation_output)
```
<!--more-->
回答：

```python
{
  "categories": {
    "harassment": false,
    "harassment/threatening": false,
    "hate": false,
    "hate/threatening": false,
    "self-harm": false,
    "self-harm/instructions": false,
    "self-harm/intent": false,
    "sexual": false,
    "sexual/minors": false,
    "violence": false,
    "violence/graphic": false
  },
  "category_scores": {
    "harassment": 3.4429521e-09,
    "harassment/threatening": 9.538529e-10,
    "hate": 6.0008998e-09,
    "hate/threatening": 3.5339007e-10,
    "self-harm": 5.6997046e-10,
    "self-harm/instructions": 3.864466e-08,
    "self-harm/intent": 9.3394e-10,
    "sexual": 2.2777907e-07,
    "sexual/minors": 2.6869095e-08,
    "violence": 3.5471032e-07,
    "violence/graphic": 7.8637696e-10
  },
  "flagged": false
}
```

可以看到这个检测方式和检测用户输入的方式是一致的。



## 评估回答质量

我们可以询问模型对生成的结果是否满意，是否符合定义的某种标准。下面是一个例子：

```python
system_message = f"""
You are an assistant that evaluates whether \
customer service agent responses sufficiently \
answer customer questions, and also validates that \
all the facts the assistant cites from the product \
information are correct.
The product information and user and customer \
service agent messages will be delimited by \
3 backticks, i.e. ```.
Respond with a Y or N character, with no punctuation:
Y - if the output sufficiently answers the question \
AND the response correctly uses product information
N - otherwise

Output a single letter only.
"""
customer_message = f"""
tell me about the smartx pro phone and \
the fotosnap camera, the dslr one. \
Also tell me about your tvs"""
product_information = """{ "name": "SmartX ProPhone", "category": "Smartphones and Accessories", "brand": "SmartX", "model_number": "SX-PP10", "warranty": "1 year", "rating": 4.6, "features": [ "6.1-inch display", "128GB storage", "12MP dual camera", "5G" ], "description": "A powerful smartphone with advanced camera features.", "price": 899.99 } { "name": "FotoSnap DSLR Camera", "category": "Cameras and Camcorders", "brand": "FotoSnap", "model_number": "FS-DSLR200", "warranty": "1 year", "rating": 4.7, "features": [ "24.2MP sensor", "1080p video", "3-inch LCD", "Interchangeable lenses" ], "description": "Capture stunning photos and videos with this versatile DSLR camera.", "price": 599.99 } { "name": "CineView 4K TV", "category": "Televisions and Home Theater Systems", "brand": "CineView", "model_number": "CV-4K55", "warranty": "2 years", "rating": 4.8, "features": [ "55-inch display", "4K resolution", "HDR", "Smart TV" ], "description": "A stunning 4K TV with vibrant colors and smart features.", "price": 599.99 } { "name": "SoundMax Home Theater", "category": "Televisions and Home Theater Systems", "brand": "SoundMax", "model_number": "SM-HT100", "warranty": "1 year", "rating": 4.4, "features": [ "5.1 channel", "1000W output", "Wireless subwoofer", "Bluetooth" ], "description": "A powerful home theater system for an immersive audio experience.", "price": 399.99 } { "name": "CineView 8K TV", "category": "Televisions and Home Theater Systems", "brand": "CineView", "model_number": "CV-8K65", "warranty": "2 years", "rating": 4.9, "features": [ "65-inch display", "8K resolution", "HDR", "Smart TV" ], "description": "Experience the future of television with this stunning 8K TV.", "price": 2999.99 } { "name": "SoundMax Soundbar", "category": "Televisions and Home Theater Systems", "brand": "SoundMax", "model_number": "SM-SB50", "warranty": "1 year", "rating": 4.3, "features": [ "2.1 channel", "300W output", "Wireless subwoofer", "Bluetooth" ], "description": "Upgrade your TV's audio with this sleek and powerful soundbar.", "price": 199.99 } { "name": "CineView OLED TV", "category": "Televisions and Home Theater Systems", "brand": "CineView", "model_number": "CV-OLED55", "warranty": "2 years", "rating": 4.7, "features": [ "55-inch display", "4K resolution", "HDR", "Smart TV" ], "description": "Experience true blacks and vibrant colors with this OLED TV.", "price": 1499.99 }"""
q_a_pair = f"""
Customer message: ```{customer_message}```
Product information: ```{product_information}```
Agent response: ```{final_response_to_customer}```

Does the response use the retrieved information correctly?
Does the response sufficiently answer the question

Output Y or N
"""
messages = [
    {'role': 'system', 'content': system_message},
    {'role': 'user', 'content': q_a_pair}
]

response = get_completion_from_messages(messages, max_tokens=1)
print(response)
```

回答：

```python
Y
```



system_message中我们要求模型判断回复的内容是否是基于product_information中提供的内容并且足够回复用户的提问。



当然对于回答的评判标准可能是根据应用场景决定的。比如判断回答是否使用了友好的语气，是否符合品牌的准则等等。



这里模型回答了`Y`，说明模型认为回答的质量是过关的。



下面这个例子给了一个与product_information毫无关系的用户回答作为测试。

```python
another_response = "life is like a box of chocolates"
q_a_pair = f"""
Customer message: ```{customer_message}```
Product information: ```{product_information}```
Agent response: ```{another_response}```

Does the response use the retrieved information correctly?
Does the response sufficiently answer the question?

Output Y or N
"""
messages = [
    {'role': 'system', 'content': system_message},
    {'role': 'user', 'content': q_a_pair}
]

response = get_completion_from_messages(messages)
print(response)
```

回答：

```python
N
```

模型发现了这个设计的用户回答与product_information毫无关系。



这样的检测可以确保模型没有产生幻觉，也就是编造不真实的东西。



## 总结

正如你所看到的，模型可以对生成的内容质量提供反馈。你可以使用这个反馈来决定是否将输出呈现给用户或生成一个新的回答。你甚至可以尝试为每个用户查询生成多个模型回答，然后让模型选择最好的一个来展示给用户。



一般来说，使用Moderation接口检查输出是很好的做法，但是要求模型评估它自己的输出可能对特定的场景反馈有用，以确保回答质量。



实际上，在大多数情况下，检测模型回答质量可能是不必要的，尤其是当你使用像GPT-4这样更高级的模型时。



检测模型回答质量会增加系统的延迟和成本，因为你必须对模型进行额外调用。如果对某些应用程序或产品来说，错误率是0.0000001%真的很重要，那么也许你应该尝试这种方法。但总的来说，不太建议在实践中这样做。



参考:

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/7/check-outputs](https://learn.deeplearning.ai/chatgpt-building-system/lesson/7/check-outputs)



文章中不好放全部的示例代码，公众号内回复 “api” 关键字可获取本篇文章完整的示例代码（格式为ipynb）。



---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)





