[TOC]

# LeetCode

## 简单

### 买卖股票的最佳时机

```python
class Solution(object):
    def maxProfit(self, prices):
        """
        :type prices: List[int]
        :rtype: int
        """
        if len(prices) < 2:
            return 0
        profit = 0
        minimum = prices[0]
        for i in prices:
            minimum = min(i, minimum)
            profit = max(i - minimum, profit)
        return profit
```

### 买卖股票的最佳时机II

```python
class Solution(object):
    def maxProfit(self, prices):
        """
        :type prices: List[int]
        :rtype: int
        """
        profit = 0
        for i in range(1,len(prices)):
            if prices[i] > prices[i-1]:
                profit += prices[i]-prices[i-1]
        return profit
```



### 二叉搜索树的最近公共祖先

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def lowestCommonAncestor(self, root, p, q):
        """
        :type root: TreeNode
        :type p: TreeNode
        :type q: TreeNode
        :rtype: TreeNode
        """
        minm = min(p.val, q.val)
        maxm = max(p.val, q.val)
        if root == None:
            return None
        if minm <= root.val <= maxm:
            return root
        else:
            l = self.lowestCommonAncestor(root.left, p, q)
            r = self.lowestCommonAncestor(root.right, p, q)
            if l:
                return l
            if r:
                return r
```



### 叶子相似的树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def leafSimilar(self, root1, root2):
        """
        :type root1: TreeNode
        :type root2: TreeNode
        :rtype: bool
        """
        def leaf(root):
            if root:
                ret = ''
                if root.left == None and root.right == None:
                    ret += str(root.val)   
                if root.left:
                    ret += leaf(root.left)
                if root.right:
                    ret += leaf(root.right)
                return ret
            
        return leaf(root1) == leaf(root2)         
```

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def leafSimilar(self, root1, root2):
        """
        :type root1: TreeNode
        :type root2: TreeNode
        :rtype: bool
        """
        def leaf(root,ret):
            if root:
                if not root.left and not root.right:
                    ret.append(root.val)
                else:
                    leaf(root.left, ret)
                    leaf(root.right, ret)
                
        ret1 = []
        ret2 = []
        leaf(root1, ret1)
        leaf(root2, ret2)
        return  ret1 == ret2
```

### 反转链表

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def reverseList(self, head):
        """
        :type head: ListNode
        :rtype: ListNode
        """
        if head == None or head.next == None:
            return head
        newhead = None
        while head:
            tmp = head.next
            head.next = newhead
            newhead = head
            head = tmp
        return newhead
```

### 猜数字大小（二分法 重要）

```python
# The guess API is already defined for you.
# @param num, your guess
# @return -1 if my number is lower, 1 if my number is higher, otherwise return 0
# def guess(num):

class Solution(object):
    def guessNumber(self, n):
        """
        :type n: int
        :rtype: int
        """
        left = 0
        right = n
        while left <=right:
            mid = left + (right - left)//2 
            g = guess(mid)
            if g == -1:
                right = mid - 1
            elif g == 1:
                left = mid + 1
            else:
                return mid
```



### 快乐数

```python
class Solution(object):
    def isHappy(self, n):
        """
        :type n: int
        :rtype: bool
        """
        stack = []
        n = str(n)
        while int(n)!=1:
            sum = 0
            for i in n:
                sum += int(i)**2
            if sum in stack:
                return False
            stack.append(sum)
            n = str(sum)
        return True
```

### 最大连续1的个数

```python
class Solution(object):
    def findMaxConsecutiveOnes(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
#         string = ''
#         for i in nums:
#             string += str(i)
#         lis = string.split('0')
#         return len(str(max(lis)))
        res = 0
        k = 0
        for x in nums:
            if x == 1:
                k = k + 1
            else:
                if k > res:
                    res = k
                k = 0
        if k > res:
            res = k
        return res
```



### 报数

```python
class Solution(object):
    def countAndSay(self, n):
        """
        :type n: int
        :rtype: str
        """ 
        ret = '1'
        for _ in range(n-1):
            count = 0
            tmp = ret[0]
            result = ''
            for i in range(0,len(ret)):
                if tmp == ret[i]:
                    count += 1
                else:
                    result = result +str(count) + tmp 
                    tmp = ret[i]
                    count = 1
            ret = result + str(count) + tmp 
        return ret
```

### 两数之和

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

### 整数翻转

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

### 回文数

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

### 字符串转整数（atoi）

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

### 转换成小写字母

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

### 宝石与石头

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

### 二叉树的坡度

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

### 搜索插入位置

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

### 公平的糖果交换

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

### 机器人能否返回原点

```python
class Solution(object):
    def judgeCircle(self, moves):
        """
        :type moves: str
        :rtype: bool
        """
        r = moves.count('R')
        # count统计字符串中出现次数
        l = moves.count('L')
        u = moves.count('U')
        d = moves.count('D')
        # print r, l, u, d
        if r == l and u == d:
            return True
        return False
```

```python
class Solution(object):
    def judgeCircle(self, moves):
        """
        :type moves: str
        :rtype: bool
        """
        return (moves.count('R') == moves.count('L')) and (moves.count('U') == moves.count('D'))
```

### 路径总和

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def hasPathSum(self, root, sum):
        """
        :type root: TreeNode
        :type sum: int
        :rtype: bool
        """
        def path(root, sum):
            if not root:
                return False
            if sum == root.val and root.left is None and root.right is None:    
                return True
            return path(root.left, sum-root.val) or path(root.right, sum-root.val)
        return path(root, sum)
    
#         if root is None:
#             return False
#         if sum == root.val and root.left is None and root.right is None:
#             return True
#         return self.hasPathSum(root.left, sum-root.val) or self.hasPathSum(root.right, sum-root.val)
        
```

### 翻转图像

```python
class Solution(object):
    def flipAndInvertImage(self, A):
        """
        :type A: List[List[int]]
        :rtype: List[List[int]]
        """
        for i in range(len(A)):
            for j in range(len(A)/2):
                tmp = A[i][len(A)-1-j] 
                A[i][len(A)-1-j] = A[i][j]
                A[i][j] = tmp
        
        for i in range(len(A)):
            for j in range(len(A)):
                if A[i][j] == 1:
                    A[i][j] = 0 
                else:
                    A[i][j] = 1      
        return A
```

```python
class Solution(object):
    def flipAndInvertImage(self, A):
        """
        :type A: List[List[int]]
        :rtype: List[List[int]]
        """        
        for i in range(len(A)):
            A[i].reverse()
            for j in range(len(A)):
                if A[i][j] == 1:
                    A[i][j] = 0 
                else:
                    A[i][j] = 1
        return A
```

### 合并二叉树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def mergeTrees(self, t1, t2):
        """
        :type t1: TreeNode
        :type t2: TreeNode
        :rtype: TreeNode
        """
        if t1 == None and t2 == None:
            return None
        if t1 == None:
            return t2
        if t2 == None:
            return t1
        t1.val += t2.val
        t1.left = self.mergeTrees(t1.left, t2.left)
        t1.right = self.mergeTrees(t1.right, t2.right)
        return t1
```

### 找到所有数组中消失的数字

```python
class Solution(object):
    def findDisappearedNumbers(self, nums):
        """
        :type nums: List[int]
        :rtype: List[int]
        """
        ret = []
        num = set(nums)
        # set() 函数创建一个无序不重复元素集，可进行关系测试，删除重复数据，还可以计算交集、差集、并集等。
        for i in range(len(nums)):
            if i+1 not in num:
                ret.append(i+1)
        return ret
        # 一行代码
        # return list(set(range(1, len(nums)+1)) - set(nums))
```

### 删除排序链表中的重复元素

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def deleteDuplicates(self, head):
        """
        :type head: ListNode
        :rtype: ListNode
        """
        if head == None:
            return head
        lis = head
        while lis.next:
            if lis.val == lis.next.val:
                lis.next = lis.next.next
            else:
                lis =lis.next
        return head
```

### Fizz Buzz

```python
class Solution(object):
    def fizzBuzz(self, n):
        """
        :type n: int
        :rtype: List[str]
        """
        if n < 1:
            return []
        ret = []
        for i in range(1,n+1):
            if i%3 == 0 and i%5 == 0:
                ret.append("FizzBuzz")
                # continue
            elif i%3 == 0:
                ret.append("Fizz")
                # continue
            elif i%5 == 0:
                ret.append("Buzz")
                # continue
            else:
                ret.append(str(i))
        return ret
```

### 错误的集合

```python
class Solution(object):
    def findErrorNums(self, nums):
        """
        :type nums: List[int]
        :rtype: List[int]
        """
        ret = []
        nums = sorted(nums)
        for i in range(1,len(nums)):
            if nums[i] == nums[i-1]:
                break
        ret.append(nums[i])
        # print list(set(range(1,len(nums)+1))-set(nums))
        ret += list(set(range(1,len(nums)+1))-set(nums))
        return ret
```

```python
class Solution(object):
    def findErrorNums(self, nums):
        """
        :type nums: List[int]
        :rtype: List[int]
        """
        res=[]
        sum1=sum(set(nums))
        sum2=(1+len(nums))*len(nums)//2
        sum3=sum(nums)
        res.append(sum3-sum1)
        res.append(sum2-sum1)
        return res
```

### 翻转二叉树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def invertTree(self, root):
        """
        :type root: TreeNode
        :rtype: TreeNode
        """
        if root is None:
            return None
        tmp = root.left
        root.left = root.right
        root.right = tmp
        root.left = self.invertTree(root.left)
        root.right = self.invertTree(root.right)
        return root
```

### 二叉树的最大深度

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def maxDepth(self, root):
        """
        :type root: TreeNode
        :rtype: int
        """
        if root == None:
            return 0
        return 1 + max(self.maxDepth(root.left) , self.maxDepth(root.right))
```

### 修剪二叉树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def trimBST(self, root, L, R):
        """
        :type root: TreeNode
        :type L: int
        :type R: int
        :rtype: TreeNode
        """
        if root == None:
            return None
        root.left = self.trimBST(root.left, L, R)
        root.right = self.trimBST(root.right, L, R)
        if root.val < L or root.val > R:
            if root.left == None:
                root = root.right
                return root
            if root.right == None:
                root = root.left
                return root
        return root
```

### 数字的补数

```python
class Solution(object):
    def findComplement(self, num):
        """
        :type num: int
        :rtype: int
        """
        # print bin(num)[2:]
        # print int("10",2)
        n = str(bin(num)[2:])
        ret = ""
        for i in range(len(n)):
            if n[i] == '1':
                ret +='0'
            else:
                ret += '1'
        return int(ret,2)
```

### 翻转字符串

```python
class Solution(object):
    def reverseString(self, s):
        """
        :type s: str
        :rtype: str
        """
        return s[::-1]        
```

### Nim游戏

```python
class Solution(object):
    def canWinNim(self, n):
        """
        :type n: int
        :rtype: bool
        """
        return n%4 != 0
```

### 最后一个单词的长度

```python
class Solution(object):
    def lengthOfLastWord(self, s):
        """
        :type s: str
        :rtype: int
        """
        s = s[::-1]
        s = s.lstrip()
        count = 0
        for i in s:
            if i == ' ':
                break
            count += 1
        return count
```

```python
class Solution(object):
    def lengthOfLastWord(self, s):
        """
        :type s: str
        :rtype: int
        """
        a = s.strip().split(' ')
        if len(a):
            return len(a[-1])
        else:
            return 0
```

### 最长公共前缀

```python
class Solution(object):
    def longestCommonPrefix(self, strs):
        """
        :type strs: List[str]
        :rtype: str
        """
        ret = ""
        strs = sorted(strs,key = lambda i:len(i),reverse=False)
        if strs == []:
            return ""
        if len(strs) == 1:
            return strs[0]
        for i in range(len(strs[0])):
            flag = 1
            for j in range(1, len(strs)):
                if strs[0][i] != strs[j][i]:
                    flag *= -1
                    break
            if flag == 1:
                ret += strs[0][i]
            if flag == -1:
                break
        return ret
```

### 汉明距离

```python
class Solution(object):
    def hammingDistance(self, x, y):
        """
        :type x: int
        :type y: int
        :rtype: int
        """
        return bin(x^y)[2:].count('1')
```

```python
class Solution(object):
    def hammingDistance(self, x, y):
        """
        :type x: int
        :type y: int
        :rtype: int
        """
        x = str(bin(x)[2:])[::-1]
        # 二进制
        y = str(bin(y)[2:])[::-1]
        count = 0
        if len(x) > len(y):
            for j in range(len(y)):
                if x[j] != y[j]:
                    count += 1
            for j in range(len(y), len(x)):
                if x[j] == '1':
                    count += 1
        else:
            for j in range(len(x)):
                if x[j] != y[j]:
                    count += 1
            for j in range(len(x), len(y)):
                if y[j] == '1':
                    count += 1
        return count
```

### 完美数

```python
class Solution(object):
    def checkPerfectNumber(self, num):
        """
        :type num: int
        :rtype: bool
        """
        l = []
        if num % 2 != 0:
            return False
        if num <= 0:
            return False
        for i in range(1,int(math.sqrt(num))+1):
            if num % i == 0:
                l.append(i)
                l.append(num/i)
        t = list(set(l))
        t.remove(num)
        if num == sum(t):
            return True
        else:
            return False

        # return num==6 or num==28 or num==496 or num==8128 or num==33550336
```

### 有效的完全平方数

```python
class Solution(object):
    def isPerfectSquare(self, num):
        """
        :type num: int
        :rtype: bool
        """
        return num**0.5 ==int(num**0.5)
```

### Excel列表名称

```python
class Solution(object):
    def convertToTitle(self, n):
        """
        :type n: int
        :rtype: str
        """
        if n <= 0:
            return ""
        if 1<=n<=26:
            return chr(n-1+ord('A'))
        # ord() convert char to int
        # chr() convert int to char
        t = divmod(n,26)
        if t[1] == 0:
            return self.convertToTitle(t[0]-1) + 'Z'
        return self.convertToTitle(t[0]) + self.convertToTitle(t[1])
```

```python

from collections import deque
class Solution(object):
    def convertToTitle(self, n):
        """
        :type n: int
        :rtype: str
        """

        column  = deque()

        while n>0:
            n,output = divmod(n-1,26)
            column.appendleft(output)

        return "".join([chr(i+ord('A')) for i in column])
```

### 寻找比目标字母大的最小字母

```python
class Solution(object):
    def nextGreatestLetter(self, letters, target):
        """
        :type letters: List[str]
        :type target: str
        :rtype: str
        """
        letters = sorted(letters)
        for i in letters:
            if i>target:
                return i
        return letters[0]
```

### 山脉数组的峰顶索引

```python
class Solution(object):
    def peakIndexInMountainArray(self, A):
        """
        :type A: List[int]
        :rtype: int
        """
        # i = max(A)
        return A.index(max(A))
```

### 自除数

```python
class Solution(object):
    def selfDividingNumbers(self, left, right):
        """
        :type left: int
        :type right: int
        :rtype: List[int]
        """
        result = []
        for i in range(left,right+1):
            num = i
            while i > 0:
                k = i%10
                if k == 0:
                    break
                if num%k == 0:
                    i = i//10
                else:
                    break
            if i == 0:
                result.append(num)
        return result
```

### 检测大写字母

```python
class Solution(object):
    def detectCapitalUse(self, word):
        """
        :type word: str
        :rtype: bool
        """
        flag = 0
        if len(word) == 1:
            return True
        if 'A'<=word[0]<='Z' and 'a'<=word[1]<='z':
            for i in range(2, len(word)):
                if 'A'<=word[i]<='Z':
                    return False
        elif 'A'<=word[0]<='Z' and 'A'<=word[1]<='Z':
            for i in range(2, len(word)):
                if 'a'<=word[i]<='z':
                    return False
        elif 'a'<=word[0]<='z':
            for i in range(1, len(word)):
                if 'A'<=word[i]<='Z':
                    return False
        return True
```

### Excel表列序号

```python
class Solution(object):
    def titleToNumber(self, s):
        """
        :type s: str
        :rtype: int
        """
        ret = 0
        s = list(s[::-1])
        for i in range(len(s)):
            ret += (ord(s[i])-ord('A')+1)*(26**i)
        return ret 
```

### 有效的括号

```python
class Solution(object):
    def isValid(self, s):
        """
        :type s: str
        :rtype: bool
        """
        if len(s) % 2 == 1:
            return False

        d = {'{': '}', '[': ']', '(': ')'}
        stack = []
        for i in s:
            # in stack
            if i in d:
                stack.append(i)
            else:
                if not stack or d[stack.pop()] != i:
                    return False
        return stack ==[]
```

### 唯一摩尔斯密码词

```python
class Solution(object):
    def uniqueMorseRepresentations(self, words):
        """
        :type words: List[str]
        :rtype: int
        """
        dir = [".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--.."]
        # print len(dir)
        trans = []
        for i in words:
            tmp = ''
            for j in i:
                tmp += dir[ord(j)-ord('a')]
            trans.append(tmp)
        return len(set(trans))
```

### 键盘行

```python
class Solution(object):
    def findWords(self, words):
        """
        :type words: List[str]
        :rtype: List[str]
        """
        fir = "qwertyuiopQWERTYUIOP"
        sec = "asdfghjklASDFGHJKL"
        thi = "zxcvbnmZXCVBNM"
        ret = []
        for i in words:
            flag = 1
            if fir.find(i[0]) != -1:
                for j in i[1:]:
                    if fir.find(j) == -1:
                        flag = 0
                        break
                if flag:
                    ret.append(i)
            elif sec.find(i[0]) != -1:
                for j in i[1:]:
                    if sec.find(j) == -1:
                        flag = 0
                        break
                if flag:
                    ret.append(i)
            elif thi.find(i[0]) != -1:
                for j in i[1:]:
                    if thi.find(j) == -1:
                        flag = 0
                        break
                if flag:
                    ret.append(i)
        return ret        
```

```python
class Solution(object):
    def findWords(self, words):
        """
        :type words: List[str]
        :rtype: List[str]
        """
        ans=[]
        keyset=['qwertyuiop','asdfghjkl','zxcvbnm']
        for keys in keyset:
            for word in words:
                line=set(word.lower())
                if line.issubset(set(keys)):
                    ans.append(word)
        return ans
```

### 反转字符串中的单词III

```python
class Solution(object):
    def reverseWords(self, s):
        """
        :type s: str
        :rtype: str
        """
        s = s.split()
        return ' '.join([i[::-1] for i in s])
```

### 转置矩阵

```python
class Solution(object):
    def transpose(self, A):
        """
        :type A: List[List[int]]
        :rtype: List[List[int]]
        """
        ret = [[0 for _ in range(len(A))] for _ in range(len(A[0]))]
        for i in range(len(A)):
            for j in range(len(A[0])):
                ret[j][i] = A[i][j]
        return ret
```

### 各位相加

```python
class Solution(object):
    def addDigits(self, num):
        """
        :type num: int
        :rtype: int
        """
        if num/10 == 0:
            return num
        n = 0
        while num/10 != 0:
            n += num%10
            num /= 10
        n += num%10
        return self.addDigits(n)
```

```python
# 不用递归
class Solution(object):
    def addDigits(self, num):
        """
        :type num: int
        :rtype: int
        """
        s = num % 9
        return s if num==0 or s!=0 else 9  
```

### 数组拆分I

```python
class Solution(object):
    def arrayPairSum(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        nums = sorted(nums)
        ret = 0
        for i in range(len(nums)/2):
            ret += nums[i*2]
        return ret
    #   return sum(sorted(nums)[::2])
```

### N叉树的后序遍历

```python
# 递归法
"""
# Definition for a Node.
class Node(object):
    def __init__(self, val, children):
        self.val = val
        self.children = children
"""
class Solution(object):
    def postorder(self, root):
        """
        :type root: Node
        :rtype: List[int]
        """
        if not root:
            return []
        if root.children == None:
            return [root.val]
        result = []
        for child in root.children:
            result += self.postorder(child)
        result.append(root.val)
        return result
```

```python
# 迭代法
"""
# Definition for a Node.
class Node(object):
    def __init__(self, val, children):
        self.val = val
        self.children = children
"""
class Solution(object):
    def postorder(self, root):
        """
        :type root: Node
        :rtype: List[int]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            curr = stack.pop()
            ret.append(curr.val)
            if curr.children:
                # curr.children.reverse()
                stack += curr.children
        return ret[::-1]
```

### N叉树的前序遍历

```python
# 迭代法 使用栈
"""
# Definition for a Node.
class Node(object):
    def __init__(self, val, children):
        self.val = val
        self.children = children
"""
class Solution(object):
    def preorder(self, root):
        """
        :type root: Node
        :rtype: List[int]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            curr = stack.pop()
            ret.append(curr.val)
            if curr.children:
                curr.children.reverse()
                stack += curr.children
        return ret
```

### N叉树的层次遍历

```python
"""
# Definition for a Node.
class Node(object):
    def __init__(self, val, children):
        self.val = val
        self.children = children
"""
class Solution(object):
    def levelOrder(self, root):
        """
        :type root: Node
        :rtype: List[List[int]]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            l = len(stack)
            sub = []
            for i in range(l):
                curr = stack.pop(0)
                sub.append(curr.val)
                for child in curr.children:
                    stack.append(child)
            ret.append(sub)
        return ret 
```

### 二叉树的层次遍历II

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def levelOrderBottom(self, root):
        """
        :type root: TreeNode
        :rtype: List[List[int]]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            l = len(stack)
            sub = []
            # print ret
            for i in range(l):
                curr = stack.pop(0)
                sub.append(curr.val)
                if curr.left:
                    stack.append(curr.left)
                if curr.right:
                    stack.append(curr.right)
            ret.append(sub)
        return ret[::-1] 
```

### 二叉树的层平均值

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def averageOfLevels(self, root):
        """
        :type root: TreeNode
        :rtype: List[float]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            l = len(stack)
            sub = []
            # print ret
            for i in range(l):
                curr = stack.pop(0)
                sub.append(curr.val)
                if curr.left:
                    stack.append(curr.left)
                if curr.right:
                    stack.append(curr.right)
            ret.append(float(sum(sub))/len(sub))
        return ret
```

### N叉树的最大深度

```python
# map(function, iterable, ...)
"""
# Definition for a Node.
class Node(object):
    def __init__(self, val, children):
        self.val = val
        self.children = children
"""
class Solution(object):
    def maxDepth(self, root):
        """
        :type root: Node
        :rtype: int
        """
        if not root:
            return 0
        if not root.children:
            return 1
        return 1 + max(list(map(self.maxDepth, root.children)))
```

### 按奇偶校验排序数组

```python
class Solution(object):
    def sortArrayByParity(self, A):
        """
        :type A: List[int]
        :rtype: List[int]
        """
        ret = []
        for i in A:
            if i%2 == 0:
                ret.append(i)
        for i in A:
            if i%2 == 1:
                ret.append(i)
        return ret
```

### 子域名访问计数

```python
class Solution(object):
    def subdomainVisits(self, cpdomains):
        """
        :type cpdomains: List[str]
        :rtype: List[str]
        """
        dicts={}       
        n = len(cpdomains)
        for item in cpdomains:
            array = item.split(' ')
            num = (int)(array[0])
            domain = array[1]
            domainLen = len(domain.split('.'))
            for i in range(domainLen):
                tmp = domain.split('.',i).pop()
                # split()通过指定分隔符对字符串进行切片，如果参数num 有指定值，则仅分隔 num 个子字符串
                if tmp in dicts.keys():
                    dicts[tmp]=dicts[tmp]+num
                else:
                    dicts[tmp]=num
        domainList=[]
        for key in dicts:
            tmp = (str)(dicts[key])+" "+key
            domainList.append(tmp)
        
        return domainList
```

### 字符的最短距离

```python
class Solution(object):
    def shortestToChar(self, S, C):
        """
        :type S: str
        :type C: str
        :rtype: List[int]
        """
        ret = []
        for i in range(len(S)):
            l,r = S.rfind(C,0,i), S.find(C,i+1,len(S))
            if S[i] == C:
                ret.append(0)
            elif l != -1:
                if r != -1:
                    # print i-S.find(C,0,i), S.find(C,i,len(S))-i
                    ret.append(min(i-l, r-i))
                else:
                    ret.append(i-l)
            elif r != -1:
                if l != -1:
                    ret.append(min(i-l, r-i))
                else:
                    ret.append(r-i)
            elif l == -1 and r == -1:
                ret.append(inf)
        return ret
```

```python
class Solution(object):
    def shortestToChar(self, S, C):
        """
        :type S: str
        :type C: str
        :rtype: List[int]
        """
        res = []
        for i in range(len(S)):
            if S[i] == C: 
                res.append(0)
                continue
                
            r = S.find(C, i+1)
            l = S.rfind(C, 0, i) if i != 0 else -1
            if l != -1 and r != -1:
                res.append(min(i - l,r - i))
            else:
                res.append(i - l) if r == -1 else res.append(r - i)
                
        return res
```

Note: **res.append(i - l) if r == -1 else res.append(r - i)**

### 将有序数组转换为二叉搜索树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def sortedArrayToBST(self, nums):
        """
        :type nums: List[int]
        :rtype: TreeNode
        """
        if not nums:
            return None
        mid=len(nums)//2
        
        root = TreeNode(nums[mid])
        root.left = self.sortedArrayToBST(nums[:mid])
        root.right = self.sortedArrayToBST(nums[mid+1:])
        return root
```

### 交替位二进制数

```python
class Solution(object):
    def hasAlternatingBits(self, n):
        """
        :type n: int
        :rtype: bool
        """
        s = bin(n)[2:]
        for i in range(len(s)-1):
            if s[i] == s[i+1]:
                return False
        return True
```

### 写字符串所需要的行数

```python
class Solution(object):
    def numberOfLines(self, widths, S):
        """
        :type widths: List[int]
        :type S: str
        :rtype: List[int]
        """
        row = 1
        count = 0
        for i in S:
            if count + widths[ord(i)-ord('a')]>100:
                count = widths[ord(i)-ord('a')]
                row += 1
            else:
                count += widths[ord(i)-ord('a')]
        return [row, count]
```

### 杨辉三角

```python
class Solution(object):
    def generate(self, numRows):
        """
        :type numRows: int
        :rtype: List[List[int]]
        """
        ret = []
        for i in range(numRows):
            tmp = [1]*(i+1)
            ret.append(tmp)
            for j in range(1,i):
                ret[i][j] = ret[i-1][j-1] + ret[i-1][j]
        return ret
```

### 杨辉三角II

```python
class Solution(object):
    def getRow(self, rowIndex):
        """
        :type rowIndex: int
        :rtype: List[int]
        """
        ret = [1]
        if not rowIndex:
            return ret
        for j in range(rowIndex):
            ret = [1] + [ret[i]+ret[i+1] for i in range(len(ret)-1)] +[1]
        return ret
```

### 重塑矩阵

```python
class Solution(object):
    def matrixReshape(self, nums, r, c):
        """
        :type nums: List[List[int]]
        :type r: int
        :type c: int
        :rtype: List[List[int]]
        """
        if len(nums)*len(nums[0]) != r*c:
            return nums
        ret = [[0 for _ in range(c)] for _ in range(r)]
        # print ret
        tmp = []
        for i in range(len(nums)):
            for j in range(len(nums[0])):
                tmp.append(nums[i][j])
        count = 0
        for i in range(r):
            for j in range(c):
                ret[i][j] = tmp[count]
                count += 1
        return ret
```

### 岛屿的周长

```python
class Solution(object):
    def islandPerimeter(self, grid):
        """
        :type grid: List[List[int]]
        :rtype: int
        """
        count = 0
        m = len(grid)
        n = len(grid[0])
        for i in range(m):
            for j in range(n):
                if grid[i][j] == 1:
                    count += 4
                    # if i-1>=0:
                    #     if grid[i-1][j] == 1:
                    #         count -= 1
                    # if j-1>=0:
                    #     if grid[i][j-1] == 1:
                    #         count -= 1
                    # if j+1<n:
                    #     if grid[i][j+1] == 1:
                    #         count -= 1
                    # if i+1<m:
                    #     if grid[i+1][j] == 1:
                    #         count -= 1
                    if i - 1 >= 0 and grid[i - 1][j] == 1:
                        count -= 2
                    if j - 1 >= 0 and grid[i][j - 1] == 1:
                        count -= 2
        return count
```

### 三位形体投影面积

```python
class Solution(object):
    def projectionArea(self, grid):
        """
        :type grid: List[List[int]]
        :rtype: int
        """
        count = 0
        m = len(grid)
        n = len(grid[0])
        for i in range(m):
            count += max(grid[i])
            count += max([j[i] for j in grid])
            # 二维列表 列
            for j in range(n):
                if grid[i][j] != 0:
                    count += 1
        return count        
```

### 罗马数字转整数

```python
class Solution(object):
    def romanToInt(self, s):
        """
        :type s: str
        :rtype: int
        """
        dir = {'I':1,'V':5,'X':10,'L':50,'C':100,'D':500,'M':1000,
               'IV':4,'IX':9,'XL':40,'XC':90,'CD':400,'CM':900}
        count = 0
        i = 0
        while i<len(s):
            if i == len(s)-1:
                count += dir[s[i]]
            else:
                key = s[i]+s[i+1]
                if dir.has_key(key):
                    count += dir[s[i]+s[i+1]]
                    i += 1
                else:
                    count += dir[s[i]]
            i += 1
        return count
```

### 二进制间距

```python
class Solution(object):
    def binaryGap(self, N):
        """
        :type N: int
        :rtype: int
        """
        s = str(bin(N)[2:])
        i = 0
        tmp = 0
        m = 0
        print s
        while i<len(s):
            if s[i] == '1':
                for j in range(i+1,len(s)):
                    if s[j] == '1':
                        tmp = j-i
                        i = j-1
                        break
                if tmp > m :
                    m = tmp
            i += 1
        return m
```

### 分糖果

```python
class Solution(object):
    def distributeCandies(self, candies):
        """
        :type candies: List[int]
        :rtype: int
        """
        return min(len(candies)//2, len(set(candies)))
```

### 用栈实现队列

```python
class MyQueue(object):

    def __init__(self):
        """
        Initialize your data structure here.
        """
        self.queue = []

    def push(self, x):
        """
        Push element x to the back of queue.
        :type x: int
        :rtype: void
        """
        self.queue.append(x)

    def pop(self):
        """
        Removes the element from in front of queue and returns that element.
        :rtype: int
        """
        return self.queue.pop(0x)

    def peek(self):
        """
        Get the front element.
        :rtype: int
        """
        return self.queue[0]

    def empty(self):
        """
        Returns whether the queue is empty.
        :rtype: bool
        """
        return len(self.queue) == 0


# Your MyQueue object will be instantiated and called as such:
# obj = MyQueue()
# obj.push(x)
# param_2 = obj.pop()
# param_3 = obj.peek()
# param_4 = obj.empty()
```

### 只出现一次的数字

```python
class Solution(object):
    def singleNumber(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        nums = sorted(nums)
        # i = 0
        l = len(nums)
        if l == 1:
            return nums[0]
        for i in range(0,l):
            if i == l-1:
                if nums[i] != nums[i-1]:
                    return nums[i]
            elif nums[i] != nums[i+1]:
                if i == 0 or nums[i] != nums[i-1]:
                    return nums[i]
            # i += 2
        return None
```

```python
class Solution(object):
    def singleNumber(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        result = 0
        for num in nums:
            result = result ^ num
            # ^与运算
        return result
```

### 下一个更大元素I

```python
class Solution(object):
    def nextGreaterElement(self, findNums, nums):
        """
        :type findNums: List[int]
        :type nums: List[int]
        :rtype: List[int]
        """
        def findbig(nums, num):
            for i in range(len(nums)):
                if nums[i]>num:
                    return nums[i]
            return -1
        ret = []
        for i in range(len(findNums)):
            loc = nums.index(findNums[i])
            ret.append(findbig(nums[loc+1:],findNums[i]))
        
        return ret
```



```python
class Solution(object):
    def nextGreaterElement(self, findNums, nums):
        """
        :type findNums: List[int]
        :type nums: List[int]
        :rtype: List[int]
        """
        # 用字典保存nums2中的{n, greater than n}，用单调栈生成dict
        greater_dict = {} # {n, greater than n}
        stack = [] # 单调栈、栈底最大
        for i in range(len(nums)):
            while stack and stack[-1] < nums[i]:
                greater_dict[stack.pop()] = nums[i]
            stack.append(nums[i])
        r = []
        for n in findNums:
            r.append(greater_dict.get(n, -1))
        return r
```

### 链表的中间节点

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def middleNode(self, head):
        """
        :type head: ListNode
        :rtype: ListNode
        """
        def length(head):
            if head == None:
                return 0
            count = 0
            while head:
                count += 1
                head = head.next
            return count
        
        mid = length(head)//2 
        while mid>0:
            head = head.next
            mid -= 1
        return head
```

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def middleNode(self, head):
        """
        :type head: ListNode
        :rtype: ListNode
        """
        slow = head
        fast = head
        while fast and fast.next:
            slow = slow.next
            fast = fast.next.next
        return slow
```

### 二叉树的所有路径

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def binaryTreePaths(self, root):
        """
        :type root: TreeNode
        :rtype: List[str]
        """
        path = ''
        res = []
        self.TreePathsHelper(root, path, res)
        return res

    def TreePathsHelper(self, root, path, res):
        if root is None:
            return
        path += str(root.val)
        if root.left != None:
            self.TreePathsHelper(root.left, path + '->', res)
        if root.right != None:
            self.TreePathsHelper(root.right, path + '->', res)
        if root.left is None and root.right is None:
            res.append(path)
```

### 找不同

```python
class Solution(object):
    def findTheDifference(self, s, t):
        """
        :type s: str
        :type t: str
        :rtype: str
        """
        s = sorted(s)
        t = sorted(t)
        for i in range(len(t)):
            if i >= len(s):
                return t[i]
            elif t[i] != s[i]:
                return t[i]
        return None
```

```python
class Solution(object):
    def findTheDifference(self, s, t):
        """
        :type s: str
        :type t: str
        :rtype: str
        """
        n = ''
        for i in set(t):
            n += ''.join(i)
        for j in n:
            if s.count(j) != t.count(j):
                return j
```

### 两个数组的交集

```python
class Solution(object):
    def intersection(self, nums1, nums2):
        """
        :type nums1: List[int]
        :type nums2: List[int]
        :rtype: List[int]
        """
        return list(set(nums1)&set(nums2))
```

### 求众数

```python
class Solution(object):
    def majorityElement(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        a = list(set(nums))
        for i in a:
            if nums.count(i)>len(nums)//2:
                return i
        
        # return sorted(nums)[len(nums)/2]
```

### 棒球比赛

```python
class Solution(object):
    def calPoints(self, ops):
        """
        :type ops: List[str]
        :rtype: int
        """
        record = 0
        stack = []
        for i in ops:
            if i == "+":
                tmp = stack[-1] + stack[-2]
                record += tmp
                stack.append(tmp)
            elif i == 'C':
                record -= stack.pop()
            elif i == 'D':
                tmp = stack[-1]*2
                record += tmp
                stack.append(tmp)
            else:
                record += int(i)
                stack.append(int(i))
        return record
```

### 二进制表示中质数个计算置位

```python
class Solution(object):
    def countPrimeSetBits(self, L, R):
        """
        :type L: int
        :type R: int
        :rtype: int
        """
        # 质数判断
        def isPrime(num):
            if num > 1:
                for i in range(2,num):
                    if (num % i) == 0:
                        return False
                return True
            else:
                return False
        count = 0
        for i in range(L,R+1):
            count += 1 if isPrime(str(bin(i)[2:]).count("1")) else 0
        return count
```

### 二叉搜索树中的搜索

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def searchBST(self, root, val):
        """
        :type root: TreeNode
        :type val: int
        :rtype: TreeNode
        """
        if root == None:
            return None
        if root.val == val:
            return root
        elif root.val < val:
            return self.searchBST(root.right, val)
        else:
            return self.searchBST(root.left, val)
```

### 存在重复元素

```python
class Solution(object):
    def containsDuplicate(self, nums):
        """
        :type nums: List[int]
        :rtype: bool
        """
        return len(nums) != len(set(nums))
```

### 2的幂

```python
class Solution(object):
    def isPowerOfTwo(self, n):
        """
        :type n: int
        :rtype: bool
        """
        if n < 1:
            return False
        return not(n & (n-1))
```

```python
class Solution(object):
    def isPowerOfTwo(self, n):
        """
        :type n: int
        :rtype: bool
        """
        while n%2 == 0 and n>1:
            n = n/2
        return (n==1)
```

### 3的幂

```python
class Solution(object):
    def isPowerOfThree(self, n):
        """
        :type n: int
        :rtype: bool
        """
        while n%3 == 0 and n>1:
            n = n/3
        return (n==1)
```

```python
class Solution:
    def isPowerOfThree(self, n):
        """
        :type n: int
        :rtype: bool
        """
        if n <= 0:
            return False
        maxint = 0x7fffffff

        k=math.log(maxint)//math.log(3)
        b3=3**k
        return (b3%n)==0
    #如果某个数n为3的幂 ，则k=log3N
	# 代码思路：
	# 首先求出int范围最大的3的幂   Max3
	# 如果n为3的幂，则Max3必定能整除n

```

### 4的幂

```python
class Solution(object):
    def isPowerOfFour(self, num):
        """
        :type num: int
        :rtype: bool
        """
        if num<=0:
            return False;
        return num & num-1==0 and num&0x55555555 ==num ;
    # 如果不能用递归循环做，就使用位操作。1个数是2的幂肯定是4的幂，但反过来不成立，4的幂只能是奇数位为1，而2的幂只有有一个位置为1就行。
# 所以先判断是否为2的幂，然后通过与.0X55555555(....1010101)进行&操作，保留奇数位，判断是否改变。
```

### 移动零

```python
class Solution(object):
    def moveZeroes(self, nums):
        """
        :type nums: List[int]
        :rtype: void Do not return anything, modify nums in-place instead.
        """
        # 移动非零元素（操作次数就是非零元素的个数）
        j = 0   # 记录非零元素应该换到第几个位置
        for i in range(len(nums)):
            if nums[i] != 0:       
                nums[j], nums[i] = nums[i], nums[j]
                j += 1
```

### 移除元素

```python
class Solution(object):
    def removeElement(self, nums, val):
        """
        :type nums: List[int]
        :type val: int
        :rtype: int
        """
        if len(nums) == 0:
            return 0;
        j=0
        for i in range(0,len(nums)):
            if nums[i] != val :
                nums[j] = nums[i]
                j = j + 1
        return j
# 题目中说不用考虑数组中超出新长度后面的元素，就是把需要的元素移到前面
```

### 删除排序数组中的重复项

```python
class Solution(object):
    def removeDuplicates(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        if len(nums) == 0:
            return 0;
        j=0
        for i in range(0,len(nums)):
            if nums[i] != nums[j]:
                j = j + 1
                nums[j] = nums[i] 
        return j+1
```



## 中等

### 删除排序数组中的重复项II

```python
class Solution(object):
    def removeDuplicates(self, nums):
        """
        :type nums: List[int]
        :rtype: int
        """
        # if len(nums) == 0:
        #     return 0
        # j = 0
        # for i in range(1,len(nums)):
        #     if nums[i] != nums[j]:
        #         j = j + 1
        #         nums[j] = nums[i] 
        #     else:
        #         if j <1:
        #             j += 1
        #         elif nums[j] != nums[j-1]:
        #             j+=1
        #             nums[j] = nums[i]          
        # return j+1
        i = 0
        for e in nums:
            if i < 2 or e != nums[i-2]:
                nums[i] = e
                i += 1
        return i
```



### 求众数II

```python
class Solution(object):
    def majorityElement(self, nums):
        """
        :type nums: List[int]
        :rtype: List[int]
        """
        a = list(set(nums))
        ret = []
        for i in a:
            if nums.count(i)>len(nums)//3:
                ret.append(i)
        return ret
```

### 两数相加

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

### 保持城市天际线

```python
class Solution(object):
    def maxIncreaseKeepingSkyline(self, grid):
        """
        :type grid: List[List[int]]
        :rtype: int
        """
        topdown = []
        leri = []
        # print len(grid)
        for i in range(len(grid[0])):
            leri.append(max(grid[i]))
        # print leri
        # for i in range(len(grid)):
        #     m = 0
        #     for j in range(len(grid[0])):
        #         if grid[j][i]>m:
        #             m = grid[j][i]
        #     topdown.append(m)
        for j in range(len(grid)):
            topdown.append(max([i[j] for i in grid ]))
        # print topdown
        sum = 0
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                sum += min(topdown[i], leri[j]) - grid[i][j]
        return sum
        
```



### 排序链表

时间复杂度在O(nlogN)的排序算法是快速排序，堆排序，归并排序。对于数组来说占用的空间复杂度为O(1),O(n),O(n)，但对于链表来说归并排序占用空间为O(1)。

快排的最坏时间复杂度是O(n^2)，平均时间复杂度为O(nlogn)。

```python
# Definition for singly-linked list.
# class ListNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution(object):
    def sortList(self, head):
        """
        :type head: ListNode
        :rtype: ListNode
        """
        if head is None or head.next is None:
            return head
        mid = self.get_mid(head)
        l = head
        r = mid.next
        mid.next = None
        return self.merge(self.sortList(l), self.sortList(r))
    
    def merge(self, p, q):
        tmp = ListNode(0)
        h = tmp
        while p and q:
            if p.val < q.val:
                h.next = p
                p = p.next
            else:
                h.next = q
                q = q.next
            h = h.next
        if p:
            h.next = p
        if q:
            h.next = q    
        return tmp.next
        
    def get_mid(self, node):
        if node is None:
            return node
        fast = slow = node
        while fast.next and fast.next.next:
            slow = slow.next
            fast = fast.next.next
        return slow
```

### 二叉树剪枝

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def pruneTree(self, root):
        """
        :type root: TreeNode
        :rtype: TreeNode
        """
        if root == None:
            return None
        # print root.val
        root.left = self.pruneTree(root.left)
        root.right = self.pruneTree(root.right)
        # 先进行递归
        if root.left == None and root.right == None :
            if root.val==0:
                return None
            else:
                return root
        return root
```

### 旋转图像

```python
class Solution(object):
    def rotate(self, matrix):
        """
        :type matrix: List[List[int]]
        :rtype: void Do not return anything, modify matrix in-place instead.
        """
        for i in range(len(matrix)-1):
            for j in range(i,len(matrix)-1-i):
                tmp1 = matrix[i][j]
                tmp2 = matrix[j][len(matrix)-1-i]
                tmp3 = matrix[len(matrix)-1-i][len(matrix)-1-j]
                tmp4 = matrix[len(matrix)-1-j][i]
                matrix[j][len(matrix)-1-i] = tmp1
                matrix[len(matrix)-1-i][len(matrix)-1-j] = tmp2
                matrix[len(matrix)-1-j][i] = tmp3
                matrix[i][j] = tmp4
                # print tmp1, tmp2, tmp3, tmp4
```

```python
class Solution(object):
    def rotate(self, matrix):
        """
        :type matrix: List[List[int]]
        :rtype: void Do not return anything, modify matrix in-place instead.
        """
   
        for i in range(len(matrix)/2):
            t = matrix[i]
            matrix[i] = matrix[len(matrix)-1-i]
            matrix[len(matrix)-1-i] = t
        l = len(matrix)
        for i in range(0,l):
            for j in range(i+1,l):
                t = matrix[i][j]
                matrix[i][j] = matrix[j][i]
                matrix[j][i] = t
```

### 最大二叉树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def constructMaximumBinaryTree(self, nums):
        """
        :type nums: List[int]
        :rtype: TreeNode
        """
        if len(nums) == 0:
            return None
        tree = TreeNode(max(nums))
        tree.left = self.constructMaximumBinaryTree(nums[:nums.index(tree.val)])
        tree.right = self.constructMaximumBinaryTree(nums[nums.index(tree.val)+1:])
        return tree
```

### 螺旋矩阵II  （不会）

```python
class Solution(object):
    def generateMatrix(self, n):
        """
        :type n: int
        :rtype: List[List[int]]
        """
        direction=0
        
        up=left=0
        down,right=n-1,n-1
        # 创建二维list
        matrix=[[0 for i in range(n)] for j in range(n)]
        
        #生成器
        nums=range(1,n**2+1)
        item=0
        #向ans里按螺旋顺序添加
        while up<=down and left<=right:
            
            if direction==0:
                for i in range(left,right+1):
                    #up在这里表示填充的这一行的行数
                    matrix[up][i]=nums[item]
                    item+=1
                up+=1
            elif direction==1:
                for i in range(up,down+1):
                    #right表示填充的这一行的列数
                    matrix[i][right]=nums[item]
                    item+=1
                right-=1
            elif direction==2:
                for i in range(left,right+1)[::-1]:
                    #down表示填充的这一行的行数
                    matrix[down][i]=nums[item]
                    item+=1
                down-=1
            else:
                for i in range(up,down+1)[::-1]:
                    #left表示填充这一行的列数
                    matrix[i][left]=nums[item]
                    item+=1
                left+=1
            direction=(direction+1)%4
        return matrix        
```

### 子集

```python

class Solution(object):
    def subsets(self, nums):
        """
        :type nums: List[int]
        :rtype: List[List[int]]
        """
        global out,s
        s = []
        out = [[]]
        def dfs(i):
            global out,s
            for j in range(i,len(nums)):
                s.append(nums[j])
                out.append(s[:])
                dfs(j+1)
                s = s[:len(s)-1] 
        dfs(0)
        return out
```

```python

class Solution(object):
    def subsets(self, nums):
        """
        :type nums: List[int]
        :rtype: List[List[int]]
        """
        res = [[]]
        for num in nums :
            for temp in res[:] :
                x = temp[:]
                x.append(num)
                res.append(x)                
        return res
```

### 整数替换 

```python
class Solution(object):
    def integerReplacement(self, n):
        """
        :type n: int
        :rtype: int
        """
        if n == 1:
            return 0
        if n%2 == 0:
            return self.integerReplacement(n/2)+1
        if n == 65535:
            return self.integerReplacement(n-1)
        else:
            return min(self.integerReplacement(n-1)+1,self.integerReplacement(n+1)+1)
```

### 逃脱阻碍者

```python
class Solution(object):
    def escapeGhosts(self, ghosts, target):
        """
        :type ghosts: List[List[int]]
        :type target: List[int]
        :rtype: bool
        """
        min = 100000
        for ghost in ghosts:
            tmp = abs(target[0] - ghost[0]) + abs(target[1] - ghost[1])
            if min > tmp:
                min = tmp
        
        if min < abs(target[0])+abs(target[1]):
            return False
        return True
```

### 二叉树的层次遍历

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def levelOrder(self, root):
        """
        :type root: TreeNode
        :rtype: List[List[int]]
        """
        if not root:
            return []
        stack = []
        ret = []
        stack.append(root)
        while stack:
            l = len(stack)
            sub = []
            # print ret
            for i in range(l):
                curr = stack.pop(0)
                sub.append(curr.val)
                if curr.left:
                    stack.append(curr.left)
                if curr.right:
                    stack.append(curr.right)
            ret.append(sub)
        return ret 
```



## 困难

### 最长有效括号(不会)

```python
class Solution:
    def longestValidParentheses(self, s):
        """
        :type s: str
        :rtype: int
        """
        tl = len(s)
        stack = []
        st = 0
        maxlen = 0
        for i in range(tl):
            #如果是左括号，直接入stack
            if s[i] == '(':
                stack.append(i)
            #如果右括号
            else:
                #如果stack里没有元素匹对，说明有效括号已经结束，更新起始位置
                if len(stack) == 0:
                    st = i+1
                    continue
                #有元素匹对
                else:
                    a = stack.pop()
                    #pop出一个左括号匹对
                    #如果此时没了，不能保证不继续有效括号，所以根据当前的最长距离去更新maxlen
                    if len(stack) == 0:
                        maxlen = max(i - st+1, maxlen)
                    #如果此时还有 则计算与栈顶的索引相减来计算长度
                    else:
                        maxlen = max(i-stack[-1], maxlen)
		return maxlen
```


