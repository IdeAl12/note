[TOC]

# LeetCode

## 两数之和（简单）

```python
class Solution(object):
    def twoSum(self, nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: List[int]
        """
        for i in range(len(nums)):
            for j in range(i+1,len(nums)):
                if nums[j] == target - nums[i]:
                    ret = [i, j]
                    return ret
```

```c
int* twoSum(int* nums, int numsSize, int target) { 
    static int ret[2] = {0,0};
    for(int i=0;i<numsSize;i++)
    {
        for(int j=i+1;j<numsSize;j++)
        {
            if(nums[i]+nums[j] == target)
            {
                ret[0]=i;
                ret[1]=j;
                return ret;
            }
        }
    }
    return ret;
}
```

## 两数相加（中等）

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def addTwoNumbers(self, l1, l2):
        """
        :type l1: ListNode
        :type l2: ListNode
        :rtype: ListNode
        """
        a1 = 0
        a2 = 0
        m1 = 1
        m2 = 1
        
        while l1:
            a1 = a1 + l1.val * m1
            m1 *= 10
            l1 = l1.next
        while l2:
            a2 = a2 + l2.val * m2
            m2 *= 10
            l2 = l2.next
        r = a1 + a2
        ret = ListNode(0)
        head = ret
        tail = ListNode(0)
        while 1:
            head.val = r%10
            head.next = ListNode(0)
            r /= 10
            if r == 0:
                head.next = None
                break
            head = head.next
        return ret
```

## 整数翻转（简单）

```python
class Solution(object):
    def reverse(self, x):
        """
        :type x: int
        :rtype: int
        """

        tmp = divmod(abs(x), 10)
        ret = 0
        while tmp[0] != 0:
            ret = ret * 10 + tmp[1] * 10
            tmp = divmod(tmp[0],10)
            # print tmp
        ret += tmp[1]
        if x < 0:
            ret = -ret
        if ret < -2**31 or ret > 2**31 - 1:
            ret = 0
        return ret
```

## 回文数（简单）

```python
class Solution(object):
    def isPalindrome(self, x):
        """
        :type x: int
        :rtype: bool
        """
        def reverse(x):
            tmp = divmod(abs(x), 10)
            ret = 0
            while tmp[0] != 0:
                ret = ret * 10 + tmp[1] * 10
                tmp = divmod(tmp[0],10)
            ret += tmp[1]
            if x < 0:
                ret = -ret
            if ret < -2**31 or ret > 2**31 - 1:
                ret = 0
            return ret
    
        if x < 0:
            return False
        revx = reverse(x)
        if x == revx:
            return True
        else:
            return False
```

