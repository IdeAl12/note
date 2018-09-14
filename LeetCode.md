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

## 字符串转整数（atoi）（简单）

```python
class Solution(object):
    def myAtoi(self, str):
        """
        :type str: str
        :rtype: int
        """
        if not str:
            return 0
        str = str.strip()
        # 用于移除字符串头尾指定的字符（默认为空格或换行符）或字符序列。
        if length == 0:
            return 0  
        num, flag = 0, 1
        if str[0] == '-':
            str = str[1:]
            flag = -1
        elif str[0] == '+':
            str = str[1:]
        for i in str:
            if i >= '0' and i <= '9':
                num = 10*num + ord(i) - ord('0')
                # ord以一个字符（长度为1的字符串）作为参数，返回对应的 ASCII 数值，或者 Unicode 数值
            else:
                break
        num *= flag
        num = num if num <= 2147483647 else 2147483647
        num = num if num >= -2147483648 else -2147483648
        return num
```

```python
# 使用正则表达式 http://www.runoob.com/python/python-reg-expressions.html
class Solution(object):
    def myAtoi(self, str):
        """
        :type str: str
        :rtype: int
        """
        str = str.strip()
        try:
            res = re.search('(^[\+\-]?\d+)', str).group()
            res = int(res)
            res = res if res <= 2147483647 else 2147483647
            res = res if res >= -2147483648 else -2147483648
        except:
            res = 0
        return res
```

## 转换成小写字母（简单）

```python
class Solution(object):
    def toLowerCase(self, str):
        """
        :type str: str
        :rtype: str
        """
        ret = ''
        if str == '':
            return Null
        for i in str:
            if i >= 'A' and i <= 'Z':
                i = chr(ord(i) + 32)
            ret += i
        #str = str.lower()
        return ret
```

## 宝石与石头（简单）

```python
class Solution(object):
    def numJewelsInStones(self, J, S):
        """
        :type J: str
        :type S: str
        :rtype: int
        """
        count = 0
        for s in S:
            if s in J:
                count += 1
        return count
```

## 二叉树的坡度（简单）

```python
# 递归
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def findTilt(self, root):
        """
        :type root: TreeNode
        :rtype: int
        """
        def sum_slope(root):
            if not root:
                return 0, 0
            sum_left, slope_left = sum_slope(root.left)
            sum_right, slope_right = sum_slope(root.right)
            return sum_left + sum_right + root.val, abs(sum_left - sum_right) + slope_left + slope_right
        
        sum, slope = sum_slope(root)
        return slope
```

## 搜索插入位置（简单）

```python
class Solution(object):
    def searchInsert(self, nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: int
        """
        count = 0
        for i in nums:
            if target <= i :
                return count
            count += 1
        return count
```



```python
class Solution(object):
    def searchInsert(self, nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: int
        """
        if target < nums[0]:
            return 0
        if target > nums[-1]:
            return len(nums)
        l = 0
        r = len(nums) - 1
        while l <= r:
            mid = (l + r) / 2
            if nums[mid] == target or (nums[mid] > target and nums[mid-1] < target):
                return mid
            if nums[mid] > target:
                r = mid -1
            if nums[mid] < target:
                l = mid + 1
```

## 公平的糖果交换（简单）

```python
class Solution(object):
    def fairCandySwap(self, A, B):
        """
        :type A: List[int]
        :type B: List[int]
        :rtype: List[int]
        """
        sum_A, sum_B = sum(A), sum(B)
        diff = (sum_B - sum_A) / 2
        setB = set(B)
        for x in A:
            if  x+diff in setB:
                return [x, x+diff]
```

