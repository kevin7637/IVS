def solution(keymap, targets):
    answer = []
    dict_key = {}
    for i in range(len(keymap)):
        for index, value in enumerate(keymap[i]):
            if value in dict_key:
                dict_key[value] = min(index, dict_key[value])
            else:
                dict_key[value] = index
    for word in targets:
        result = 0
        for spell in word:
            if spell not in dict_key:
                result = -1
                break
            else:
                result += (dict_key[spell] + 1)
        answer.append(result)
    return answer
