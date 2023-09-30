---
title: 如何使用ChatGPT的API(二)分类用户输入
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: 5d0abf76
date: 2023-09-30 20:07:16
---

对于一个机器人问答系统，用户的提问多种多样。为了更好的应对用户的提问，将用户的提问进行分类，然后根据不同的提问类别针对性地回答问题。这样系统会显得更智能。



在篇文章中，我们将重点关注如何对用户的输入进行分类。这对于确保系统的质量和安全性非常重要。对于需要处理具有大量独立指令集的任务，首先对问题类型进行分类，然后根据分类结果确定使用哪些指令，这对整个问答系统是非常有益的。


<!--more-->
我们可以设定一些固定的类别以及在各个类别下绑定相关的任务指令。例如，在构建客户服务助手时，首先对问题类型进行分类，然后根据分类确定使用哪些指令。比如用户要求关闭他们的账户和用户询问特定产品是不同类别的问题，需要不同的指令来处理。



让我们看一个例子，以便大家更加清楚地理解。

```python
delimiter = "####"
system_message = f"""
You will be provided with customer service queries. \
The customer service query will be delimited with \
{delimiter} characters.
Classify each query into a primary category \
and a secondary category. 
Provide your output in json format with the \
keys: primary and secondary.

Primary categories: Billing, Technical Support, \
Account Management, or General Inquiry.

Billing secondary categories:
Unsubscribe or upgrade
Add a payment method
Explanation for charge
Dispute a charge

Technical Support secondary categories:
General troubleshooting
Device compatibility
Software updates

Account Management secondary categories:
Password reset
Update personal information
Close account
Account security

General Inquiry secondary categories:
Product information
Pricing
Feedback
Speak to a human

"""
user_message = f"""\
I want you to delete my profile and all of my user data"""
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
{
  "primary": "Account Management",
  "secondary": "Close account"
}
```

这个例子中设定了系统消息，它是整个系统的指令。这里也使用了“####”作为分隔符。分隔符是一种分隔指令，它有助于模型确定不同的部分。“####”是一个很好的分隔符，因为它实际上刚好被划分为一个Token。而一个Token刚好是模型的最小处理单元。



这里使用分隔符将用户的提问隔离开来。然后也说明了要以JSON格式提供输出，其中key为primary和secondary。系统消息中列出了主要类别。如，Billing, Technical Support, Account Management, or General Inquiry。然后在下面列出了主要类别下面的次要类别。



然后用户的输入是，“我想要删除我的个人资料和所有用户数据。这看起来像是账户管理的操作，也许是关闭账户。



让我们看看模型的回答。很好，模型的分类是账户管理作为主要类别，关闭账户作为次要类别。



说明模型正确地将用户的输入进行了分类。



这里结构化的输出（JSON）是非常有用的。我们可以将模型的回答数据轻松地读入某种对象中，例如Python中的字典，然后将其用于后续步骤的处理。



下面这个例子是另一个用户消息，“告诉我更多关于你们的平板电视。



这里模型给出的第二个分类是产品信息，看起来是正确的。因此，根据客户输入的分类，我们可以提供一组更具体的指令来进行下一步处理。在这种情况下，我们可能会添加有关电视的附加信息，而不是给出关闭账户的链接之类的。



```python
user_message = f"""\
Tell me more about your flat screen tvs"""
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
{
  "primary": "General Inquiry",
  "secondary": "Product information"
}
```



参考：

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/3/classification](https://learn.deeplearning.ai/chatgpt-building-system/lesson/3/classification)



文章中不好放全部的示例代码，公众号内回复 “api” 关键字可获取本篇文章完整的示例代码（格式为ipynb）。



---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

