---
title: 如何使用ChatGPT的API(四)思维链推理
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: f527c2fe
date: 2023-09-30 20:23:16
---


在回答一个具体问题之前，模型对问题进行详细的推理是很重要的。有时，模型可能会因为急于得出结论而犯推理错误，所以我们可以仔细设计prompt，要求在模型提供最终答案之前进行一系列相关的推理步骤，这样它就可以更长时间、更有条理地思考这个问题。



而像这样要求模型分步骤推理一个问题的策略被称为**思考链推理**。

<!--more-->

## 思维链推理

下面的`system_message`内容演示了让模型进行思维链推理的方法。

```python
delimiter = "####"
system_message = f"""
Follow these steps to answer the customer queries.
The customer query will be delimited with four hashtags,\
i.e. {delimiter}. 

Step 1:{delimiter} First decide whether the user is \
asking a question about a specific product or products. \
Product cateogry doesn't count. 

Step 2:{delimiter} If the user is asking about \
specific products, identify whether \
the products are in the following list.
All available products: 
1. Product: TechPro Ultrabook
   Category: Computers and Laptops
   Brand: TechPro
   Model Number: TP-UB100
   Warranty: 1 year
   Rating: 4.5
   Features: 13.3-inch display, 8GB RAM, 256GB SSD, Intel Core i5 processor
   Description: A sleek and lightweight ultrabook for everyday use.
   Price: $799.99

2. Product: BlueWave Gaming Laptop
   Category: Computers and Laptops
   Brand: BlueWave
   Model Number: BW-GL200
   Warranty: 2 years
   Rating: 4.7
   Features: 15.6-inch display, 16GB RAM, 512GB SSD, NVIDIA GeForce RTX 3060
   Description: A high-performance gaming laptop for an immersive experience.
   Price: $1199.99

3. Product: PowerLite Convertible
   Category: Computers and Laptops
   Brand: PowerLite
   Model Number: PL-CV300
   Warranty: 1 year
   Rating: 4.3
   Features: 14-inch touchscreen, 8GB RAM, 256GB SSD, 360-degree hinge
   Description: A versatile convertible laptop with a responsive touchscreen.
   Price: $699.99

4. Product: TechPro Desktop
   Category: Computers and Laptops
   Brand: TechPro
   Model Number: TP-DT500
   Warranty: 1 year
   Rating: 4.4
   Features: Intel Core i7 processor, 16GB RAM, 1TB HDD, NVIDIA GeForce GTX 1660
   Description: A powerful desktop computer for work and play.
   Price: $999.99

5. Product: BlueWave Chromebook
   Category: Computers and Laptops
   Brand: BlueWave
   Model Number: BW-CB100
   Warranty: 1 year
   Rating: 4.1
   Features: 11.6-inch display, 4GB RAM, 32GB eMMC, Chrome OS
   Description: A compact and affordable Chromebook for everyday tasks.
   Price: $249.99

Step 3:{delimiter} If the message contains products \
in the list above, list any assumptions that the \
user is making in their \
message e.g. that Laptop X is bigger than \
Laptop Y, or that Laptop Z has a 2 year warranty.

Step 4:{delimiter}: If the user made any assumptions, \
figure out whether the assumption is true based on your \
product information. 

Step 5:{delimiter}: First, politely correct the \
customer's incorrect assumptions if applicable. \
Only mention or reference products in the list of \
5 available products, as these are the only 5 \
products that the store sells. \
Answer the customer in a friendly tone.

Use the following format:
Step 1:{delimiter} <step 1 reasoning>
Step 2:{delimiter} <step 2 reasoning>
Step 3:{delimiter} <step 3 reasoning>
Step 4:{delimiter} <step 4 reasoning>
Response to 
user:{delimiter} <response to customer>

Make sure to include {delimiter} to separate every step.
"""
```

这个例子中设定了5个步骤让模型去思考，通过让模型分步骤思考推理，以便得出更为准确的回答。

例子中设定了5款笔记本电脑产品。如果有用户来询问产品信息，模型要分步骤思考并正确回答用户的问题。

步骤一中，首先确定用户是否在询问特定产品的问题。

步骤二中，列出了所有的电脑产品及其相关的信息。并要求模型判断用户询问的产品是否在列表中。

步骤三中，要求模型列出用户对产品提出的假设。比如：这款电脑比那款尺寸大。

步骤四中，要求模型确定用户对产品的认知是否正确。

步骤五中，如果用户对产品的认知有错误，要求模型礼貌地提醒用户。



**注意**，这里有意用分隔符隔开了模型思考推理的过程和最终的回答。目的是方便后续提取出模型最终的回答，而模型推理过程中输出的内容并不需要展示给用户。



下面是一个测试，来看看模型是否能正确处理用户的提问。

```python
user_message = f"""
by how much is the BlueWave Chromebook more expensive \
than the TechPro Desktop"""

messages =  [  
{'role':'system', 
 'content': system_message},    
{'role':'user', 
 'content': f"{delimiter}{user_message}{delimiter}"},  
] 

response = get_completion_from_messages(messages)
print(response)
```

回答:

```python
Step 1:#### The user is asking about the price difference between the BlueWave Chromebook and the TechPro Desktop.

Step 2:#### Both the BlueWave Chromebook and the TechPro Desktop are available products.

Step 3:#### The user assumes that the BlueWave Chromebook is more expensive than the TechPro Desktop.

Step 4:#### Based on the product information, the price of the BlueWave Chromebook is $249.99, and the price of the TechPro Desktop is $999.99. Therefore, the TechPro Desktop is actually more expensive than the BlueWave Chromebook.

Response to user:#### The BlueWave Chromebook is actually less expensive than the TechPro Desktop. The BlueWave Chromebook is priced at $249.99, while the TechPro Desktop is priced at $999.99.
```

这里用户的提问是`BlueWave Chromebook`比`TechPro Desktop`贵多少。事实上，`BlueWave Chromebook`是比`TechPro Desktop`便宜的。



可以看到，模型分步骤进行了推理。步骤4中，模型正确地推理出用户的假设是错误的，并且告诉用户`TechPro Desktop`是更贵的。`Response to use`中将最终的结论输出给了用户。



另外一个例子，我们问问是否卖电视。

```python
user_message = f"""
do you sell tvs"""
messages =  [  
{'role':'system', 
 'content': system_message},    
{'role':'user', 
 'content': f"{delimiter}{user_message}{delimiter}"},  
] 
response = get_completion_from_messages(messages)
print(response)
```

回答：

```python
Step 1:#### The user is asking if the store sells TVs, which is a question about a specific product category.

Step 2:#### TVs are not included in the list of available products. The store only sells computers and laptops.

Response to user:#### I'm sorry, but we currently do not sell TVs. Our store specializes in computers and laptops. If you have any questions or need assistance with our available products, feel free to ask.
```

由于电视不在产品列表中，模型直接跳过了步骤3和步骤4，表现不错。这个行为是符合步骤3中写明的判断的，即判断产品是否在列表中。如果用户询问的产品不在产品列表中也就无需步骤3和步骤4的输出了。



由于我们要求模型用一个分隔符来分隔其推理步骤，所以我们可以将思维链推理过程隐藏起来，让用户只看到最终输出。

```python
try:
    final_response = response.split(delimiter)[-1].strip()
except Exception as e:
    final_response = "Sorry, I'm having trouble right now, please try asking another question."
    
print(final_response)
```

结果：

```python
I'm sorry, but we currently do not sell TVs. Our store specializes in computers and laptops. If you have any questions or need assistance with our available products, feel free to ask.
```

上述代码只提取出最终的结果并输出给用户。模型中间的推理过程没有必要让用户看到。



参考：

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/5/chain-of-thought-reasoning](https://learn.deeplearning.ai/chatgpt-building-system/lesson/5/chain-of-thought-reasoning)



文章中不好放全部的示例代码，公众号内回复 “api” 关键字可获取本篇文章完整的示例代码（格式为ipynb）。



---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

