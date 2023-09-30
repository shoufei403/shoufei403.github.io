---
title: 如何使用ChatGPT的API(八)评估提示词
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: fc465e61
date: 2023-09-30 20:09:16
---

之前的文章展示了如何使用大语言模型构建应用程序，从评估输入、处理输入到在向用户显示输出之前进行最终输出检查。



在构建了这样一个系统之后，怎么知道它是如何工作的呢？甚至在部署系统并让用户使用时，如何跟踪系统的运行情况，找出不足之处，并继续提高系统答案的质量呢？



这篇文章将与大家分享一些评估大语言模型输出的最佳实践。

<!--more-->

这里需要说明一下基于提示词构建AI应用与基于传统机器学习监督学习构建应用的区别。

![](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/image-20230628235514926.png)



基于提示词的开发将模型开发的核心部分从数月缩短到几天，甚至几分钟或几小时。



而在传统的监督学习方法中，无论如何都需要先收集比如说 10,000 个标注示例，那么再收集 1,000 个测试示例。因此，在传统的监督学习中，收集一个训练集、收集一个开发集或测试集中的保留交叉验证集，然后在整个开发过程中利用收集的这些例子是很常见的。但这将耗费大量时间。



在使用大语言模型构建应用程序时，只需在一小部分示例上调整提示词，可能是 1 到 3 到 5 个示例，并尝试在这些示例上获得有效的提示词。然后，在对系统进行更多测试时，偶尔会遇到一些棘手的例子。提示词对它们不起作用，或者算法对它们不起作用。



在这种情况下，你可以把这些额外的一个、两个、三个或五个例子添加到你正在测试的集合中，以便适时添加额外的棘手例子。最终，你会有足够多的例子被添加到你缓慢增长的开发集中，以至于每次更改提示词时都要手动运行每个例子变得有点不方便。



然后，你开始制定衡量标准来衡量这一小部分示例的性能，例如平均准确率。这个过程还有一个有趣的方面，那就是如果你在任何时候认为你的系统运行得足够好，你就可以就此打住。



现在，如果正在评估模型的手工构建开发集还不能让你对系统性能有足够的信心，那么这时你就可以进入下一步，收集一组随机抽样的示例来调整模型。这仍将是一个开发集或交叉验证集，因为在此基础上继续调整模型是很常见的。只有当你需要对系统性能进行更高保真度的评估时，你才可能收集并使用一个测试集。因此，如果你的系统在 91% 的情况下都能给出正确答案，而你希望对系统进行调整，使其在 92% 或 93% 的情况下都能给出正确答案，那么收集测试集就显得更为重要，因为你确实需要更大的示例集来测量 91% 和 93% 性能之间的差异。



只有当你真的需要对系统的表现进行公正、公平的评估时，你才需要在开发集之外再收集一个测试集。一个重要的注意事项是，对于很多大型语言模型的应用，如果给出的答案不完全正确，也不会有任何伤害和风险。

 

但很明显，对于任何高风险的应用，如果存在偏差或不适当的输出对他人造成伤害的风险，那么收集测试集以严格评估系统性能就变得更加重要了。但是，举例来说，如果你只是用它来总结文章，供自己阅读，而不供其他人阅读，那么伤害的风险可能就比较小，你就可以在这个过程中尽早停止，而不必花费更多的代价，收集更大的数据集来评估你的算法。



下面我们以之前文章中构建的客户助手作为例子来演示如何评估大语言模型应用。



## 找相关产品和类别

```python
products_and_category = utils.get_products_and_category()
products_and_category
```

输出：

```python

{'Computers and Laptops': ['TechPro Ultrabook',
  'BlueWave Gaming Laptop',
  'PowerLite Convertible',
  'TechPro Desktop',
  'BlueWave Chromebook'],
 'Smartphones and Accessories': ['SmartX ProPhone',
  'MobiTech PowerCase',
  'SmartX MiniPhone',
  'MobiTech Wireless Charger',
  'SmartX EarBuds'],
 'Televisions and Home Theater Systems': ['CineView 4K TV',
  'SoundMax Home Theater',
  'CineView 8K TV',
  'SoundMax Soundbar',
  'CineView OLED TV'],
 'Gaming Consoles and Accessories': ['GameSphere X',
  'ProGamer Controller',
  'GameSphere Y',
  'ProGamer Racing Wheel',
  'GameSphere VR Headset'],
 'Audio Equipment': ['AudioPhonic Noise-Canceling Headphones',
  'WaveSound Bluetooth Speaker',
  'AudioPhonic True Wireless Earbuds',
  'WaveSound Soundbar',
  'AudioPhonic Turntable'],
 'Cameras and Camcorders': ['FotoSnap DSLR Camera',
  'ActionCam 4K',
  'FotoSnap Mirrorless Camera',
  'ZoomMaster Camcorder',
  'FotoSnap Instant Camera']}
```

这里使用 utils 中的辅助函数获取产品和类别列表。因此，在电脑和笔记本电脑类别中，这里有一个电脑和笔记本电脑的列表；在智能手机和配件类别中，这里有一个智能手机和配件的列表；其他类别也是如此。



```python
def find_category_and_product_v1(user_input,products_and_category):

    delimiter = "####"
    system_message = f"""
    You will be provided with customer service queries. \
    The customer service query will be delimited with {delimiter} characters.
    Output a python list of json objects, where each object has the following format:
        'category': <one of Computers and Laptops, Smartphones and Accessories, Televisions and Home Theater Systems, \
    Gaming Consoles and Accessories, Audio Equipment, Cameras and Camcorders>,
    AND
        'products': <a list of products that must be found in the allowed products below>


    Where the categories and products must be found in the customer service query.
    If a product is mentioned, it must be associated with the correct category in the allowed products list below.
    If no products or categories are found, output an empty list.
    

    List out all products that are relevant to the customer service query based on how closely it relates
    to the product name and product category.
    Do not assume, from the name of the product, any features or attributes such as relative quality or price.

    The allowed products are provided in JSON format.
    The keys of each item represent the category.
    The values of each item is a list of products that are within that category.
    Allowed products: {products_and_category}
    

    """
    
    few_shot_user_1 = """I want the most expensive computer."""
    few_shot_assistant_1 = """ 
    [{'category': 'Computers and Laptops', \
'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]
    """
    
    messages =  [  
    {'role':'system', 'content': system_message},    
    {'role':'user', 'content': f"{delimiter}{few_shot_user_1}{delimiter}"},  
    {'role':'assistant', 'content': few_shot_assistant_1 },
    {'role':'user', 'content': f"{delimiter}{user_input}{delimiter}"},  
    ] 
    return get_completion_from_messages(messages)
```

这里构建了一个提示词版本。在这个提示词中也添加了样本来供大语言模型学习。系统消息中则添加了产品类别信息作为信息来源。下面我们测试几个示例来看看效果。

```python
customer_msg_0 = f"""Which TV can I buy if I'm on a budget?"""

products_by_category_0 = find_category_and_product_v1(customer_msg_0,
                                                      products_and_category)
print(products_by_category_0)
```

回答：

```python
[{'category': 'Televisions and Home Theater Systems', 'products': ['CineView 4K TV', 'SoundMax Home Theater', 'CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV']}]
```
用户的提问为：如果我的预算有限，可以买什么电视机？
这里检索用户提问相关的类别和产品，以便我们有正确的信息来回答用户的询问。

```python
customer_msg_1 = f"""I need a charger for my smartphone"""

products_by_category_1 = find_category_and_product_v1(customer_msg_1,
                                                      products_and_category)
print(products_by_category_1)
```

回答：

```python
[{'category': 'Smartphones and Accessories', 'products': ['MobiTech PowerCase', 'MobiTech Wireless Charger', 'SmartX EarBuds']}]
```

```python
customer_msg_2 = f"""
What computers do you have?"""

products_by_category_2 = find_category_and_product_v1(customer_msg_2,
                                                      products_and_category)
products_by_category_2
```

回答：

```python

" \n    [{'category': 'Computers and Laptops', 'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]"
```
对于这个例子，回答中出现了不需要的\n字符。

```python
customer_msg_3 = f"""
tell me about the smartx pro phone and the fotosnap camera, the dslr one.
Also, what TVs do you have?"""

products_by_category_3 = find_category_and_product_v1(customer_msg_3,
                                                      products_and_category)
print(products_by_category_3)
```

回答：

```python
[{'category': 'Smartphones and Accessories', 'products': ['SmartX ProPhone']}, {'category': 'Cameras and Camcorders', 'products': ['FotoSnap DSLR Camera']}, {'category': 'Televisions and Home Theater Systems', 'products': ['CineView 4K TV', 'SoundMax Home Theater', 'CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV']}]
```

## 更难的测试

```python
customer_msg_4 = f"""
tell me about the CineView TV, the 8K one, Gamesphere console, the X one.
I'm on a budget, what computers do you have?"""

products_by_category_4 = find_category_and_product_v1(customer_msg_4,
                                                      products_and_category)
print(products_by_category_4)
```

回答：

```python
[{'category': 'Televisions and Home Theater Systems', 'products': ['CineView 8K TV']}, {'category': 'Gaming Consoles and Accessories', 'products': ['GameSphere X']}, {'category': 'Computers and Laptops', 'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]
```


## 修改提示词以应对更难的测试
```python
def find_category_and_product_v2(user_input,products_and_category):
    """
    Added: Do not output any additional text that is not in JSON format.
    Added a second example (for few-shot prompting) where user asks for 
    the cheapest computer. In both few-shot examples, the shown response 
    is the full list of products in JSON only.
    """
    delimiter = "####"
    system_message = f"""
    You will be provided with customer service queries. \
    The customer service query will be delimited with {delimiter} characters.
    Output a python list of json objects, where each object has the following format:
        'category': <one of Computers and Laptops, Smartphones and Accessories, Televisions and Home Theater Systems, \
    Gaming Consoles and Accessories, Audio Equipment, Cameras and Camcorders>,
    AND
        'products': <a list of products that must be found in the allowed products below>
    Do not output any additional text that is not in JSON format.
    Do not write any explanatory text after outputting the requested JSON.


    Where the categories and products must be found in the customer service query.
    If a product is mentioned, it must be associated with the correct category in the allowed products list below.
    If no products or categories are found, output an empty list.
    

    List out all products that are relevant to the customer service query based on how closely it relates
    to the product name and product category.
    Do not assume, from the name of the product, any features or attributes such as relative quality or price.

    The allowed products are provided in JSON format.
    The keys of each item represent the category.
    The values of each item is a list of products that are within that category.
    Allowed products: {products_and_category}
    

    """
    
    few_shot_user_1 = """I want the most expensive computer. What do you recommend?"""
    few_shot_assistant_1 = """ 
    [{'category': 'Computers and Laptops', \
'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]
    """
    
    few_shot_user_2 = """I want the most cheapest computer. What do you recommend?"""
    few_shot_assistant_2 = """ 
    [{'category': 'Computers and Laptops', \
'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]
    """
    
    messages =  [  
    {'role':'system', 'content': system_message},    
    {'role':'user', 'content': f"{delimiter}{few_shot_user_1}{delimiter}"},  
    {'role':'assistant', 'content': few_shot_assistant_1 },
    {'role':'user', 'content': f"{delimiter}{few_shot_user_2}{delimiter}"},  
    {'role':'assistant', 'content': few_shot_assistant_2 },
    {'role':'user', 'content': f"{delimiter}{user_input}{delimiter}"},  
    ] 
    return get_completion_from_messages(messages)

```
之前的提示词版本已经取得了不错的效果，但对于有些例子仍存在一些瑕疵。在这个更新的版本中，我们添加了 如下语句，要求模型不要输出非JSON格式的字符。

```python
Do not output any additional text that is not in JSON format.
Do not write any explanatory text after outputting the requested JSON.
```
另外，我们补充了一个样本让模型学习。当然对于更高级的模型，如GPT-4，这些可能就不需要了。


```python
customer_msg_3 = f"""
tell me about the smartx pro phone and the fotosnap camera, the dslr one.
Also, what TVs do you have?"""

products_by_category_3 = find_category_and_product_v2(customer_msg_3,
                                                      products_and_category)
print(products_by_category_3)
```
回答：
```python
[{'category': 'Smartphones and Accessories', 'products': ['SmartX ProPhone']}, {'category': 'Cameras and Camcorders', 'products': ['FotoSnap DSLR Camera']}, {'category': 'Televisions and Home Theater Systems', 'products': ['CineView 4K TV', 'SoundMax Home Theater', 'CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV']}]
```

```python
customer_msg_4 = f"""
tell me about the CineView TV, the 8K one, Gamesphere console, the X one.
I'm on a budget, what computers do you have?"""

products_by_category_4 = find_category_and_product_v2(customer_msg_4,
                                                      products_and_category)
print(products_by_category_4)
```
回答：
```python
[{'category': 'Televisions and Home Theater Systems', 'products': ['CineView 4K TV', 'CineView 8K TV']}, {'category': 'Gaming Consoles and Accessories', 'products': ['GameSphere X']}, {'category': 'Computers and Laptops', 'products': ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook']}]
```

## 回归测试

检查修改了提示词后是否会对之前测试用例的性能产生负面影响。

```python
customer_msg_0 = f"""Which TV can I buy if I'm on a budget?"""

products_by_category_0 = find_category_and_product_v2(customer_msg_0,
                                                      products_and_category)
print(products_by_category_0)
```
回答：
```python
[{'category': 'Televisions and Home Theater Systems', 'products': ['CineView 4K TV', 'SoundMax Home Theater', 'CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV']}]
```

经过一些测试示例，我们发现现在的提示词版本已经可以正确地根据用户输入找到相关的产品信息，并且以JSON格式输入相关产品信息。

下面是更为严谨的一步，设定一个测试集来评估系统的性能。


## 基于测试集的自动测试

这里设定一个测试集，其包含用户的输入和最佳回答。
```python
msg_ideal_pairs_set = [
    
    # eg 0
    {'customer_msg':"""Which TV can I buy if I'm on a budget?""",
     'ideal_answer':{
        'Televisions and Home Theater Systems':set(
            ['CineView 4K TV', 'SoundMax Home Theater', 'CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV']
        )}
    },

    # eg 1
    {'customer_msg':"""I need a charger for my smartphone""",
     'ideal_answer':{
        'Smartphones and Accessories':set(
            ['MobiTech PowerCase', 'MobiTech Wireless Charger', 'SmartX EarBuds']
        )}
    },
    # eg 2
    {'customer_msg':f"""What computers do you have?""",
     'ideal_answer':{
           'Computers and Laptops':set(
               ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook'
               ])
                }
    },

    # eg 3
    {'customer_msg':f"""tell me about the smartx pro phone and \
    the fotosnap camera, the dslr one.\
    Also, what TVs do you have?""",
     'ideal_answer':{
        'Smartphones and Accessories':set(
            ['SmartX ProPhone']),
        'Cameras and Camcorders':set(
            ['FotoSnap DSLR Camera']),
        'Televisions and Home Theater Systems':set(
            ['CineView 4K TV', 'SoundMax Home Theater','CineView 8K TV', 'SoundMax Soundbar', 'CineView OLED TV'])
        }
    }, 

    # eg 4
    {'customer_msg':"""tell me about the CineView TV, the 8K one, Gamesphere console, the X one.
I'm on a budget, what computers do you have?""",
     'ideal_answer':{
        'Televisions and Home Theater Systems':set(
            ['CineView 8K TV']),
        'Gaming Consoles and Accessories':set(
            ['GameSphere X']),
        'Computers and Laptops':set(
            ['TechPro Ultrabook', 'BlueWave Gaming Laptop', 'PowerLite Convertible', 'TechPro Desktop', 'BlueWave Chromebook'])
        }
    },
    
    # eg 5
    {'customer_msg':f"""What smartphones do you have?""",
     'ideal_answer':{
           'Smartphones and Accessories':set(
               ['SmartX ProPhone', 'MobiTech PowerCase', 'SmartX MiniPhone', 'MobiTech Wireless Charger', 'SmartX EarBuds'
               ])
                    }
    },
    # eg 6
    {'customer_msg':f"""I'm on a budget.  Can you recommend some smartphones to me?""",
     'ideal_answer':{
        'Smartphones and Accessories':set(
            ['SmartX EarBuds', 'SmartX MiniPhone', 'MobiTech PowerCase', 'SmartX ProPhone', 'MobiTech Wireless Charger']
        )}
    },

    # eg 7 # this will output a subset of the ideal answer
    {'customer_msg':f"""What Gaming consoles would be good for my friend who is into racing games?""",
     'ideal_answer':{
        'Gaming Consoles and Accessories':set([
            'GameSphere X',
            'ProGamer Controller',
            'GameSphere Y',
            'ProGamer Racing Wheel',
            'GameSphere VR Headset'
     ])}
    },
    # eg 8
    {'customer_msg':f"""What could be a good present for my videographer friend?""",
     'ideal_answer': {
        'Cameras and Camcorders':set([
        'FotoSnap DSLR Camera', 'ActionCam 4K', 'FotoSnap Mirrorless Camera', 'ZoomMaster Camcorder', 'FotoSnap Instant Camera'
        ])}
    },
    
    # eg 9
    {'customer_msg':f"""I would like a hot tub time machine.""",
     'ideal_answer': []
    }
    
]
```

下面的代码对比模型的回答和理想的答案。
```python
import json
def eval_response_with_ideal(response,
                              ideal,
                              debug=False):
    
    if debug:
        print("response")
        print(response)
    
    # json.loads() expects double quotes, not single quotes
    json_like_str = response.replace("'",'"')
    
    # parse into a list of dictionaries
    l_of_d = json.loads(json_like_str)
    
    # special case when response is empty list
    if l_of_d == [] and ideal == []:
        return 1
    
    # otherwise, response is empty 
    # or ideal should be empty, there's a mismatch
    elif l_of_d == [] or ideal == []:
        return 0
    
    correct = 0    
    
    if debug:
        print("l_of_d is")
        print(l_of_d)
    for d in l_of_d:

        cat = d.get('category')
        prod_l = d.get('products')
        if cat and prod_l:
            # convert list to set for comparison
            prod_set = set(prod_l)
            # get ideal set of products
            ideal_cat = ideal.get(cat)
            if ideal_cat:
                prod_set_ideal = set(ideal.get(cat))
            else:
                if debug:
                    print(f"did not find category {cat} in ideal")
                    print(f"ideal: {ideal}")
                continue
                
            if debug:
                print("prod_set\n",prod_set)
                print()
                print("prod_set_ideal\n",prod_set_ideal)

            if prod_set == prod_set_ideal:
                if debug:
                    print("correct")
                correct +=1
            else:
                print("incorrect")
                print(f"prod_set: {prod_set}")
                print(f"prod_set_ideal: {prod_set_ideal}")
                if prod_set <= prod_set_ideal:
                    print("response is a subset of the ideal answer")
                elif prod_set >= prod_set_ideal:
                    print("response is a superset of the ideal answer")

    # count correct over total number of items in list
    pc_correct = correct / len(l_of_d)
        
    return pc_correct
```

```python
print(f'Customer message: {msg_ideal_pairs_set[7]["customer_msg"]}')
print(f'Ideal answer: {msg_ideal_pairs_set[7]["ideal_answer"]}')
```
输出：
```python
Customer message: What Gaming consoles would be good for my friend who is into racing games?
Ideal answer: {'Gaming Consoles and Accessories': {'ProGamer Racing Wheel', 'GameSphere Y', 'ProGamer Controller', 'GameSphere VR Headset', 'GameSphere X'}}
```

这里手动测试一下，看是否测试函数能正常工作。
```python
response = find_category_and_product_v2(msg_ideal_pairs_set[7]["customer_msg"],
                                         products_and_category)
print(f'Resonse: {response}')

eval_response_with_ideal(response,
                              msg_ideal_pairs_set[7]["ideal_answer"])
```
回答：
```python

Resonse:  
    [{'category': 'Gaming Consoles and Accessories', 'products': ['GameSphere X', 'ProGamer Controller', 'GameSphere Y', 'ProGamer Racing Wheel', 'GameSphere VR Headset']}]
    
1.0
```


下面是基于测试集进行的自动化测试并输出测试结果。
```python
# Note, this will not work if any of the api calls time out
score_accum = 0
for i, pair in enumerate(msg_ideal_pairs_set):
    print(f"example {i}")
    
    customer_msg = pair['customer_msg']
    ideal = pair['ideal_answer']
    
    # print("Customer message",customer_msg)
    # print("ideal:",ideal)
    response = find_category_and_product_v2(customer_msg,
                                                      products_and_category)

    
    # print("products_by_category",products_by_category)
    score = eval_response_with_ideal(response,ideal,debug=False)
    print(f"{i}: {score}")
    score_accum += score
    

n_examples = len(msg_ideal_pairs_set)
fraction_correct = score_accum / n_examples
print(f"Fraction correct out of {n_examples}: {fraction_correct}")
```
回答:
```python
example 0
0: 1.0
example 1
incorrect
prod_set: {'MobiTech PowerCase', 'SmartX ProPhone', 'MobiTech Wireless Charger', 'SmartX EarBuds', 'SmartX MiniPhone'}
prod_set_ideal: {'MobiTech PowerCase', 'MobiTech Wireless Charger', 'SmartX EarBuds'}
response is a superset of the ideal answer
1: 0.0
example 2
2: 1.0
example 3
3: 1.0
example 4
incorrect
prod_set: {'SoundMax Home Theater', 'SoundMax Soundbar', 'CineView 4K TV', 'CineView OLED TV', 'CineView 8K TV'}
prod_set_ideal: {'CineView 8K TV'}
response is a superset of the ideal answer
incorrect
prod_set: {'ProGamer Racing Wheel', 'GameSphere Y', 'ProGamer Controller', 'GameSphere VR Headset', 'GameSphere X'}
prod_set_ideal: {'GameSphere X'}
response is a superset of the ideal answer
4: 0.3333333333333333
example 5
5: 1.0
example 6
6: 1.0
example 7
7: 1.0
example 8
8: 1.0
example 9
9: 1
Fraction correct out of 10: 0.8333333333333334
```



## 总结

使用提示词构建应用程序的工作流程与使用监督学习构建应用程序的工作流程截然不同，而且迭代的速度感觉要快得多。如果你以前没有做过，你可能会惊讶于建立在几个手工编辑的棘手示例基础上的评估方法。你用 10 个例子来思考，这在统计学上几乎对任何事情都是无效的。但当你真正使用这个程序时，你可能会惊讶地发现，在开发集中添加少量、仅仅是少量棘手的示例，对于帮助你和你的团队获得一套有效的提示和有效的系统是多么有效。在本篇文章示例中，输出结果是可以量化评估的，以便评估系统是否达到了理想状态。



参考：

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/9/evaluation-part-i](https://learn.deeplearning.ai/chatgpt-building-system/lesson/9/evaluation-part-i)



文章中不好放全部的示例代码，公众号内《首飞》回复 “api” 关键字可获取本篇文章完整的示例代码（格式为ipynb）。





---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



