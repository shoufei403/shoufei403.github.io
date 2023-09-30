---
title: 如何使用ChatGPT的API(七)一个客户服务助手
categories: ChatGPT
tags:
  - ChatGPT
  - OpenAI
abbrlink: 7bcceabb
date: 2023-09-30 20:05:16
---

在本篇文章中，我们将综合前面文章中所有知识，创建一个端到端的客户服务助理示例。我们将经历以下步骤：

首先，我们将通过Moderation API检查输入是否违规。

其次，如果没有，我们将提取产品列表。

第三，如果找到产品信息，我们将尝试查找它们。

第四，我们用模型回答用户的问题。

第五，我们将通过Moderation API对答案进行审核。如果回答没有违规，我们可以把它返回给用户。

第六，对模型的回答进行质量评估

<!--more-->

示例代码：

```python
def process_user_message(user_input, all_messages, debug=True):
    delimiter = "```"
    
    # Step 1: Check input to see if it flags the Moderation API or is a prompt injection
    response = openai.Moderation.create(input=user_input)
    moderation_output = response["results"][0]

    if moderation_output["flagged"]:
        print("Step 1: Input flagged by Moderation API.")
        return "Sorry, we cannot process this request."

    if debug: print("Step 1: Input passed moderation check.")
    
    category_and_product_response = utils.find_category_and_product_only(user_input, utils.get_products_and_category())
    #print(print(category_and_product_response)
    # Step 2: Extract the list of products
    category_and_product_list = utils.read_string_to_list(category_and_product_response)
    #print(category_and_product_list)

    if debug: print("Step 2: Extracted list of products.")

    # Step 3: If products are found, look them up
    product_information = utils.generate_output_string(category_and_product_list)
    if debug: print("Step 3: Looked up product information.")

    # Step 4: Answer the user question
    system_message = f"""
    You are a customer service assistant for a large electronic store. \
    Respond in a friendly and helpful tone, with concise answers. \
    Make sure to ask the user relevant follow-up questions.
    """
    messages = [
        {'role': 'system', 'content': system_message},
        {'role': 'user', 'content': f"{delimiter}{user_input}{delimiter}"},
        {'role': 'assistant', 'content': f"Relevant product information:\n{product_information}"}
    ]

    final_response = get_completion_from_messages(all_messages + messages)
    if debug:print("Step 4: Generated response to user question.")
    all_messages = all_messages + messages[1:]

    # Step 5: Put the answer through the Moderation API
    response = openai.Moderation.create(input=final_response)
    moderation_output = response["results"][0]

    if moderation_output["flagged"]:
        if debug: print("Step 5: Response flagged by Moderation API.")
        return "Sorry, we cannot provide this information."

    if debug: print("Step 5: Response passed moderation check.")

    # Step 6: Ask the model if the response answers the initial user query well
    user_message = f"""
    Customer message: {delimiter}{user_input}{delimiter}
    Agent response: {delimiter}{final_response}{delimiter}

    Does the response sufficiently answer the question?
    """
    messages = [
        {'role': 'system', 'content': system_message},
        {'role': 'user', 'content': user_message}
    ]
    evaluation_response = get_completion_from_messages(messages)
    if debug: print("Step 6: Model evaluated the response.")

    # Step 7: If yes, use this answer; if not, say that you will connect the user to a human
    if "Y" in evaluation_response:  # Using "in" instead of "==" to be safer for model output variation (e.g., "Y." or "Yes")
        if debug: print("Step 7: Model approved the response.")
        return final_response, all_messages
    else:
        if debug: print("Step 7: Model disapproved the response.")
        neg_str = "I'm unable to provide the information you're looking for. I'll connect you with a human representative for further assistance."
        return neg_str, all_messages

user_input = "tell me about the smartx pro phone and the fotosnap camera, the dslr one. Also what tell me about your tvs"
response,_ = process_user_message(user_input,[])
print(response)
```


上面的示例代码中，我们正在按步骤回答用户问题。第一步是审核输入，第二步是提取产品列表。第三步是查询产品信息。

当有了产品信息，模型则根据产品信息回答用户的问题。最后，它将回答再次给到Moderation API，以确保可以安全地显示给用户。

当然，这里也添加了让模型去评估回答质量的功能。

这些是我们之前文章知识的一个汇总。



提取产品信息的辅助函数`utils.find_category_and_product_only`, `utils.read_string_to_list`和`utils.generate_output_string`这里没有列出来。可以在公众号《首飞》内输入“api”查看到完整的源码。



参考：

[https://learn.deeplearning.ai/chatgpt-building-system/lesson/8/evaluation](https://learn.deeplearning.ai/chatgpt-building-system/lesson/8/evaluation)



---

**觉得有用就点个赞吧！**

我是首飞，一个帮大家填坑的工程师。

![公众号](https://sf-blog-images.oss-cn-hangzhou.aliyuncs.com/shoufei_qr_gongzhonghao.jpg)



