---
title: ChatGPT提示词攻略(七)之制作一个聊天机器人
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: c538d306
date: 2023-06-22 09:27:16
---

大型语言模型的一个令人兴奋的方面是，你可以利用它来构建一个定制的聊天机器人，并且只需付出少量的努力。ChatGPT 的网页界面可以让你与一个大型语言模型进行对话。但其中一个很酷的功能是，你也可以利用大型语言模型构建你自己的定制聊天机器人，例如扮演一个人工智能客服代理或餐厅的人工智能点餐员的角色。本篇文章将揭晓如何做到这一点。



下面是两种调用OpenAI接口的函数。`get_completion`只支持单轮对话，意味着模型回答问题时不会考虑之前的对话信息。`get_completion_from_messages`支持多轮对话信息。这是因为它的参数`message`可以包含对话的上下文。下面我们看看它们在对话机器人中是怎么使用的。
<!--more-->
```python
# 支持单轮对话
def get_completion(prompt, model="gpt-3.5-turbo"):
    messages = [{"role": "user", "content": prompt}]
    response = openai.ChatCompletion.create(
        model=model,
        messages=messages,
        temperature=0, # this is the degree of randomness of the model's output
    )
    return response.choices[0].message["content"]

# 支持多轮对话
def get_completion_from_messages(messages, model="gpt-3.5-turbo", temperature=0):
    response = openai.ChatCompletion.create(
        model=model,
        messages=messages,
        temperature=temperature, # this is the degree of randomness of the model's output
    )
    #print(str(response.choices[0].message))  #该打印可以打印完整的返回信息
    return response.choices[0].message["content"]
```



## 角色定义

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230617101859426.png)

下面是一个消息列表的例子，首先是一个**系统消息**，它提供了整体指示。在这条消息之后，我们有**用户**和**助手**之间的多轮对话，这个对话会一直进行下去。如果你曾经使用过ChatGPT的网页界面，那么你的消息就是**用户消息**，ChatGPT的消息就是**助手消息**。**系统消息有助于设定助手的行为和个性**，并且它在对话中充当高级指令。你可以将其视为在助手耳边私下交流并引导其回复，而用户并不知道系统消息的存在。作为用户，如果你曾经使用过ChatGPT，你可能不知道ChatGPT的系统消息内容。系统消息的好处在于，它为开发者提供了一种在对话中框定对话范围的方式，而不用让这些框定的话语成为对话的一部分。这样，你可以引导助手，私下指导其回复，而不让用户察觉。



```python
messages =  [  
{'role':'system', 'content':'You are an assistant that speaks like Shakespeare.'},    
{'role':'user', 'content':'tell me a joke'},   
{'role':'assistant', 'content':'Why did the chicken cross the road'},   
{'role':'user', 'content':'I don\'t know'}  ]

response = get_completion_from_messages(messages, temperature=1)
print(response)
```

回答：

```python
#response.choices[0].message 中的完整内容
{
  "content": "To get to the other side, my good sir!",
  "role": "assistant"
}

To get to the other side, my good sir!
```



## OpenAI服务器不会记住对话的上下文信息

下面的示例展示了OpenAI服务器不会主动记住对话的上下文信息的现象。所以要在对话中让模型根据对话的上下文来回答问题，我们需要把之前的对话一并发给OpenAI服务器。 

```python
messages =  [  
{'role':'system', 'content':'You are friendly chatbot.'},    
{'role':'user', 'content':'Hi, my name is Isa'}  ]
response = get_completion_from_messages(messages, temperature=1)
print(response)
```

回答：

```python
Hello Isa! It's great to meet you. How can I assist you today?
```

再次单独询问时，模型并没有回答出我的名字是Isa。

```python
messages =  [  
{'role':'system', 'content':'You are friendly chatbot.'},    
{'role':'user', 'content':'Yes,  can you remind me, What is my name?'}  ]
response = get_completion_from_messages(messages, temperature=1)
print(response)
```

回答：

```python
I apologize but I don't have access to your name as it is not provided to me. Can you please tell me what name I should refer you as?
```



当我们把前后文信息一起发送个模型，模型正确地回答了我的名字。

```python
messages =  [  
{'role':'system', 'content':'You are friendly chatbot.'},
{'role':'user', 'content':'Hi, my name is Isa'},
{'role':'assistant', 'content': "Hi Isa! It's nice to meet you. \
Is there anything I can help you with today?"},
{'role':'user', 'content':'Yes, you can remind me, What is my name?'}  ]
response = get_completion_from_messages(messages, temperature=1)
print(response)
```

回答：

```python
Your name is Isa. :)
```

这里也引出了一个问题。当我们不断累积上下文对话，输入的文字会越来越多。一方面模型对输入的文字数量是有限制的，另一方面每个文字可都是算钱的。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230617102644174.png)

所以，当文字多了会对上下文进行总结，或者遗忘最开始的一些对话，然后再输入给模型。当然总结的事也可以让ChatGPT干的。



不过，最近OpenAI放出了新的模型，最常用的gpt-3.5-turbo模型有了新版本，可支持的Token（近似文字数量）提升了4倍，价格还下降了25%。可喜可贺。



## 订披萨的对话机器人示例

订餐机器人(OrderBot)可以自动收集用户的订餐需求并作出回应。这个订餐机器人将会在一家比萨饼店接受用户订单。

```python
def collect_messages(_):
    prompt = inp.value_input
    inp.value = ''
    context.append({'role':'user', 'content':f"{prompt}"}) #这里收集用户的输入
    response = get_completion_from_messages(context) 
    context.append({'role':'assistant', 'content':f"{response}"}) #这里将机器人的回答加入上下文
    panels.append(
        pn.Row('User:', pn.pane.Markdown(prompt, width=600)))
    panels.append(
        pn.Row('Assistant:', pn.pane.Markdown(response, width=600, style={'background-color': '#F6F6F6'})))
 
    return pn.Column(*panels)
```



```python
import panel as pn  # GUI
pn.extension()

panels = [] # collect display 

context = [ {'role':'system', 'content':"""
You are OrderBot, an automated service to collect orders for a pizza restaurant. \
You first greet the customer, then collects the order, \
and then asks if it's a pickup or delivery. \
You wait to collect the entire order, then summarize it and check for a final \
time if the customer wants to add anything else. \
If it's a delivery, you ask for an address. \
Finally you collect the payment.\
Make sure to clarify all options, extras and sizes to uniquely \
identify the item from the menu.\
You respond in a short, very conversational friendly style. \
The menu includes \
pepperoni pizza  12.95, 10.00, 7.00 \
cheese pizza   10.95, 9.25, 6.50 \
eggplant pizza   11.95, 9.75, 6.75 \
fries 4.50, 3.50 \
greek salad 7.25 \
Toppings: \
extra cheese 2.00, \
mushrooms 1.50 \
sausage 3.00 \
canadian bacon 3.50 \
AI sauce 1.50 \
peppers 1.00 \
Drinks: \
coke 3.00, 2.00, 1.00 \
sprite 3.00, 2.00, 1.00 \
bottled water 5.00 \
"""} ]  # accumulate messages


inp = pn.widgets.TextInput(value="Hi", placeholder='Enter text here…')
button_conversation = pn.widgets.Button(name="Chat!")

interactive_conversation = pn.bind(collect_messages, button_conversation)

dashboard = pn.Column(
    inp,
    pn.Row(button_conversation),
    pn.panel(interactive_conversation, loading_indicator=True, height=300),
)

dashboard
```

上面的代码定义了system角色，并把披萨店的菜单，商品价格和回复客户的方式都定义好了。

下面的对话示例可以看到效果还是很棒的。

```python
User:

Assistant:

Hello! Welcome to our pizza restaurant. What can I get for you today?

User:

Hi! I would lile to pick up a pizza

Assistant:

Great! What kind of pizza would you like to order? We have pepperoni, cheese, and eggplant pizza.

User:

How much are they?

Assistant:

The pepperoni pizza is $12.95 for a large, $10.00 for a medium, and $7.00 for a small. The cheese pizza is $10.95 for a large, $9.25 for a medium, and $6.50 for a small. The eggplant pizza is $11.95 for a large, $9.75 for a medium, and $6.75 for a small.

User:

pepperoni pizza please

Assistant:

Great choice! What size would you like for your pepperoni pizza?

User:

large

Assistant:

Got it! Would you like any toppings on your pizza? We have extra cheese, mushrooms, sausage, Canadian bacon, AI sauce, and peppers.

User:

what is the total price?

Assistant:

The total price for a large pepperoni pizza without any toppings is $12.95. If you would like to add any toppings, I can let you know the additional cost.

User:

thank you. That’s all.

Assistant:

Alright, just to confirm, you would like to order a large pepperoni pizza without any toppings. Is that correct?

User:

yes

Assistant:

Great! Is this for pickup or delivery?

User:

pick up

Assistant:

Perfect! Your total for the large pepperoni pizza is $12.95. Is there anything else you would like to add to your order?

User:

No. Thanks.

Assistant:

You’re welcome! Please come to our restaurant for pickup in 20-25 minutes. Our address is 123 Main Street. Thank you for choosing our pizza restaurant!
```



最后将客户的订单进行总结，并按JOSN格式输出。

```python
messages =  context.copy()
messages.append(
{'role':'system', 'content':'create a json summary of the previous food order. Itemize the price for each item\
 The fields should be 1) pizza, include size 2) list of toppings 3) list of drinks, include size   4) list of sides include size  5)total price '},    
)
 #The fields should be 1) pizza, price 2) list of toppings 3) list of drinks, include size include price  4) list of sides include size include price, 5)total price '},    

response = get_completion_from_messages(messages, temperature=0)
print(response)
```

回答：

```python
Here's a JSON summary of the previous food order:

​```
{
  "pizza": {
    "type": "pepperoni",
    "size": "large",
    "price": 12.95
  },
  "toppings": [],
  "drinks": [],
  "sides": [],
  "total_price": 12.95
}
​``` 

Since the customer did not order any toppings, drinks, or sides, those fields are empty. The total price is $12.95, which is the price of the large pepperoni pizza.
```



上面定义了system角色来回答，其实这里用user效果是一样的。

```python
messages =  context.copy()
messages.append(
{'role':'user', 'content':'create a json summary of the previous food order. Itemize the price for each item\
 The fields should be 1) pizza, include size 2) list of toppings 3) list of drinks, include size   4) list of sides include size  5)total price '},    
)
 #The fields should be 1) pizza, price 2) list of toppings 3) list of drinks, include size include price  4) list of sides include size include price, 5)total price '},    

response = get_completion_from_messages(messages, temperature=0)
print(response)
```

回答：

```python
Sure, here's a JSON summary of your order:

​```
{
  "pizza": {
    "type": "pepperoni",
    "size": "large",
    "price": 12.95
  },
  "toppings": [],
  "drinks": [],
  "sides": [],
  "total_price": 12.95
}
​```

Please note that the "toppings", "drinks", and "sides" fields are empty since you did not order any. If you had ordered any toppings, drinks, or sides, they would be listed under their respective fields with their corresponding prices.
```



参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/8/chatbot](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/8/chatbot)





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)

