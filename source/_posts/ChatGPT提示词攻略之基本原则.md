---
title: ChatGPT提示词攻略之基本原则
categories: AIGC
tags:
  - ChatGPT
  - 提示词
abbrlink: ae5fa866
date: 2023-06-04 17:30:16
---


下面是调用openai的completion接口的函数。但在本文中并不是重点。了解一下就好。

```python
import openai
import os

from dotenv import load_dotenv, find_dotenv
_ = load_dotenv(find_dotenv())

openai.api_key  = os.getenv('OPENAI_API_KEY')

def get_completion(prompt, model="gpt-3.5-turbo"):
    messages = [{"role": "user", "content": prompt}]
    response = openai.ChatCompletion.create(
        model=model,
        messages=messages,
        temperature=0, # this is the degree of randomness of the model's output
    )
    return response.choices[0].message["content"]
```

下面我们来说说，书写提示词的基本原则。

<!--more-->

## 提示词的基本原则

- 提示词的书写要清晰，带有明确的指令
- 给模型时间去思考，即指明模型的思考过程



### 原则一：提示词的书写要清晰，带有明确的指令

**技巧一：使用分隔符清楚地指示输入的不同部分**

分隔符可以是```, """, < >, <tag> </tag>, :

```python
text = f"""
You should express what you want a model to do by \ 
providing instructions that are as clear and \ 
specific as you can possibly make them. \ 
This will guide the model towards the desired output, \ 
and reduce the chances of receiving irrelevant \ 
or incorrect responses. Don't confuse writing a \ 
clear prompt with writing a short prompt. \ 
In many cases, longer prompts provide more clarity \ 
and context for the model, which can lead to \ 
more detailed and relevant outputs.
"""
prompt = f"""
Summarize the text delimited by triple backticks \ 
into a single sentence.
​```{text}```
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
Clear and specific instructions should be provided to guide a model towards the desired output, and longer prompts can provide more clarity and context for the model, leading to more detailed and relevant outputs.
```

这个例子中需要处理的内容和处理指令是区分开的。这样便于维护。



**技巧二：指明固定格式输出结果，如JSON, HTML**

```python
prompt = f"""
Generate a list of three made-up book titles along \ 
with their authors and genres. 
Provide them in JSON format with the following keys: 
book_id, title, author, genre.
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
[
  {
    "book_id": 1,
    "title": "The Lost City of Zorath",
    "author": "Aria Blackwood",
    "genre": "Fantasy"
  },
  {
    "book_id": 2,
    "title": "The Last Survivors",
    "author": "Ethan Stone",
    "genre": "Science Fiction"
  },
  {
    "book_id": 3,
    "title": "The Secret of the Haunted Mansion",
    "author": "Lila Rose",
    "genre": "Mystery"
  }
]
```



**技巧三：让模型检测条件是否被满足**

示例1：

```python
text_1 = f"""
Making a cup of tea is easy! First, you need to get some \ 
water boiling. While that's happening, \ 
grab a cup and put a tea bag in it. Once the water is \ 
hot enough, just pour it over the tea bag. \ 
Let it sit for a bit so the tea can steep. After a \ 
few minutes, take out the tea bag. If you \ 
like, you can add some sugar or milk to taste. \ 
And that's it! You've got yourself a delicious \ 
cup of tea to enjoy.
"""
prompt = f"""
You will be provided with text delimited by triple quotes. 
If it contains a sequence of instructions, \ 
re-write those instructions in the following format:

Step 1 - ...
Step 2 - …
…
Step N - …

If the text does not contain a sequence of instructions, \ 
then simply write \"No steps provided.\"

\"\"\"{text_1}\"\"\"
"""
response = get_completion(prompt)
print("Completion for Text 1:")
print(response)
```

回答：

```python

Completion for Text 1:
Step 1 - Get some water boiling.
Step 2 - Grab a cup and put a tea bag in it.
Step 3 - Once the water is hot enough, pour it over the tea bag.
Step 4 - Let it sit for a bit so the tea can steep.
Step 5 - After a few minutes, take out the tea bag.
Step 6 - Add some sugar or milk to taste.
Step 7 - Enjoy your delicious cup of tea!
```

这里设定的条件是，如果文本中有指令步骤就将其按步骤列出。同时，这里也写明了输出的格式，正如技巧二中提到的那样。



示例2：

```python
text_2 = f"""
The sun is shining brightly today, and the birds are \
singing. It's a beautiful day to go for a \ 
walk in the park. The flowers are blooming, and the \ 
trees are swaying gently in the breeze. People \ 
are out and about, enjoying the lovely weather. \ 
Some are having picnics, while others are playing \ 
games or simply relaxing on the grass. It's a \ 
perfect day to spend time outdoors and appreciate the \ 
beauty of nature.
"""
prompt = f"""
You will be provided with text delimited by triple quotes. 
If it contains a sequence of instructions, \ 
re-write those instructions in the following format:

Step 1 - ...
Step 2 - …
…
Step N - …

If the text does not contain a sequence of instructions, \ 
then simply write \"No steps provided.\"

\"\"\"{text_2}\"\"\"
"""
response = get_completion(prompt)
print("Completion for Text 2:")
print(response)
```

回答：

```python
Completion for Text 2:
No steps provided.
```

示例二中的文本没有出现类似指令步骤的文字，模型也正确判断出来了。这说明模型是有推理能力的。



**技巧四：少样本学习**

```python
prompt = f"""
Your task is to answer in a consistent style.

<child>: Teach me about patience.

<grandparent>: The river that carves the deepest \ 
valley flows from a modest spring; the \ 
grandest symphony originates from a single note; \ 
the most intricate tapestry begins with a solitary thread.

<child>: Teach me about resilience.
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
<grandparent>: Resilience is like a tree that bends with the wind but never breaks. It is the ability to bounce back from adversity and keep moving forward, even when things get tough. Just like a tree that grows stronger with each storm it weathers, resilience is a quality that can be developed and strengthened over time.
```

通过给模型一个示例去学习，让模型在接下来的回答中参照示例来回答。





### 原则二：给模型时间去思考

**技巧一：指定完成任务所需的步骤**

```python
text = f"""
In a charming village, siblings Jack and Jill set out on \ 
a quest to fetch water from a hilltop \ 
well. As they climbed, singing joyfully, misfortune \ 
struck—Jack tripped on a stone and tumbled \ 
down the hill, with Jill following suit. \ 
Though slightly battered, the pair returned home to \ 
comforting embraces. Despite the mishap, \ 
their adventurous spirits remained undimmed, and they \ 
continued exploring with delight.
"""
# example 1
prompt_1 = f"""
Perform the following actions: 
1 - Summarize the following text delimited by triple \
backticks with 1 sentence.
2 - Translate the summary into Chinese.
3 - List each name in the Chinese summary.
4 - Output a json object that contains the following \
keys: chinese_summary, num_names.

Separate your answers with line breaks.

Text:
​```{text}```
"""
response = get_completion(prompt_1)
print("Completion for prompt 1:")
print(response)
```

回答：

```python
Completion for prompt 1:
1 - Siblings Jack and Jill go on a quest to fetch water from a hilltop well, but misfortune strikes as Jack trips and tumbles down the hill, with Jill following suit, yet they return home slightly battered but undeterred in their adventurous spirits. 

2 - 兄妹杰克和吉尔出发去从山顶井中取水，但不幸的是，杰克绊倒了，滚下了山坡，吉尔也跟着滚下来，但他们稍微受了点伤，回到家中得到了安慰的拥抱，尽管发生了不幸，他们的冒险精神仍然不减，继续愉快地探索。

3 - 杰克，吉尔。

4 - 
{
  "chinese_summary": "兄妹杰克和吉尔出发去从山顶井中取水，但不幸的是，杰克绊倒了，滚下了山坡，吉尔也跟着滚下来，但他们稍微受了点伤，回到家中得到了安慰的拥抱，尽管发生了不幸，他们的冒险精神仍然不减，继续愉快地探索。",
  "num_names": 2
}
```

模型按照我们制定的步骤输出了内容。



当我们重新制定输出格式

```python
prompt_2 = f"""
Your task is to perform the following actions: 
1 - Summarize the following text delimited by 
  <> with 1 sentence.
2 - Translate the summary into Chinese.
3 - List each name in the Chinese summary.
4 - Output a json object that contains the 
  following keys: chinese_summary, num_names.

Use the following format:
Text: <text to summarize>
Summary: <summary>
Translation: <summary translation>
Names: <list of names in Italian summary>
Output JSON: <json with summary and num_names>

Text: <{text}>
"""
response = get_completion(prompt_2)
print("\nCompletion for prompt 2:")
print(response)
```

注意：这里的text指的也是上面例子中的text。

回答：

```python
Completion for prompt 2:
Summary: Jack and Jill go on a quest to fetch water, but misfortune strikes and they tumble down a hill, returning home slightly battered but with undimmed adventurous spirits. 
Translation: Jack 和 Jill 去取水，但不幸的是他们跌倒了，回家时略有些受伤，但他们的冒险精神仍然不减，继续愉快地探索。
Names: Jack, Jill
Output JSON: {"chinese_summary": "Jack 和 Jill 去取水，但不幸的是他们跌倒了，回家时略有些受伤，但他们的冒险精神仍然不减，继续愉快地探索。", "num_names": 2}
```

模型真的理解了输出的格式要求，有点厉害了哦。



**技巧二：指示模型在匆忙得出结论之前制定自己的解决方案**

```python
prompt = f"""
Determine if the student's solution is correct or not.

Question:
I'm building a solar power installation and I need \
 help working out the financials. 
- Land costs $100 / square foot
- I can buy solar panels for $250 / square foot
- I negotiated a contract for maintenance that will cost \ 
me a flat $100k per year, and an additional $10 / square \
foot
What is the total cost for the first year of operations 
as a function of the number of square feet.

Student's Solution:
Let x be the size of the installation in square feet.
Costs:
1. Land cost: 100x
2. Solar panel cost: 250x
3. Maintenance cost: 100,000 + 100x
Total cost: 100x + 250x + 100,000 + 100x = 450x + 100,000
"""
response = get_completion(prompt)
print(response)
```

回答：

```python
The student's solution is correct.
```

这里学生的回答是错误的。模型却判断为正确。看样子它算数是真不好。



补救措施来了。先让模型自己找出一个方案，然后让它去和学生的解决方案进行对比。最后判断一下学生的方案是否正确。

```python
prompt = f"""
Your task is to determine if the student's solution \
is correct or not.
To solve the problem do the following:
- First, work out your own solution to the problem. 
- Then compare your solution to the student's solution \ 
and evaluate if the student's solution is correct or not. 
Don't decide if the student's solution is correct until 
you have done the problem yourself.

Use the following format:
Question:
​```
question here
​```
Student's solution:
​```
student's solution here
​```
Actual solution:
​```
steps to work out the solution and your solution here
​```
Is the student's solution the same as actual solution \
just calculated:
​```
yes or no
​```
Student grade:
​```
correct or incorrect
​```

Question:
​```
I'm building a solar power installation and I need help \
working out the financials. 
- Land costs $100 / square foot
- I can buy solar panels for $250 / square foot
- I negotiated a contract for maintenance that will cost \
me a flat $100k per year, and an additional $10 / square \
foot
What is the total cost for the first year of operations \
as a function of the number of square feet.
​``` 
Student's solution:
​```
Let x be the size of the installation in square feet.
Costs:
1. Land cost: 100x
2. Solar panel cost: 250x
3. Maintenance cost: 100,000 + 100x
Total cost: 100x + 250x + 100,000 + 100x = 450x + 100,000
​```
Actual solution:
"""
response = get_completion(prompt)
print(response)
```

回答：

```python

Let x be the size of the installation in square feet.

Costs:
1. Land cost: 100x
2. Solar panel cost: 250x
3. Maintenance cost: 100,000 + 10x

Total cost: 100x + 250x + 100,000 + 10x = 360x + 100,000

Is the student's solution the same as actual solution just calculated:
No

Student grade:
Incorrect
```

通过这么一顿操作，模型似乎领悟到了什么。它发现了学生在`Maintenance cost`中错误地将`10x`写成了`100x`。

总结一下该原则，通过设置思考过程，让模型**按步骤思考**通常能得到更为准确的回答。



## 大语言模型的限制

```python
prompt = f"""
介绍一下首飞牌电动牙刷的功能
"""
response = get_completion(prompt)
print(response)
```

回答：

```python

首飞牌电动牙刷是一款高科技的电动牙刷，具有多种功能，包括：

1. 高速震动清洁：首飞牌电动牙刷采用高速震动技术，每分钟可达到40000次的震动频率，能够深入清洁牙齿，有效去除牙菌斑和牙渍。

2. 智能感应：该电动牙刷配备了智能感应技术，能够自动识别不同的牙齿区域，根据不同的清洁需求进行智能调节，让你的牙齿更加干净。

3. 多种清洁模式：首飞牌电动牙刷提供了多种清洁模式，包括标准模式、深层清洁模式、按摩模式等，可以根据个人需求进行选择。

4. 长续航时间：该电动牙刷采用高效节能的电池技术，一次充电可以使用长达30天，让你无需频繁充电。

5. 人性化设计：首飞牌电动牙刷采用人性化设计，握感舒适，操作简单，适合各种年龄段的人使用。

总之，首飞牌电动牙刷具有高速震动清洁、智能感应、多种清洁模式、长续航时间和人性化设计等多种功能，是一款高性能的电动牙刷，可以帮助你轻松保持口腔健康。
```

事实上，并没有首飞牌电动牙刷，但模型描述的挺欢快的。一本正经地胡编乱造。这就是模型表现出来的**幻想**。要尽量避免这种情况，就需要将提示词写的准确，清晰和指令化。



参考：

[https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/2/guidelines](https://learn.deeplearning.ai/chatgpt-prompt-eng/lesson/2/guidelines)





---

**觉得有用就点个赞吧！**

我是首飞，做有趣的事情，拿出来分享。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)





