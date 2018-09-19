[TOC]

# LeetCode

## 简单
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



## 中等

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
        # 先进性递归
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



## 竞赛

### 第一次

#### 按奇偶校验排序数组（简单）成功

给定一个非负整数数组 `A`，返回一个由 `A` 的所有偶数元素组成的数组，后面跟 `A` 的所有奇数元素。

你可以返回满足此条件的任何数组作为答案。 

**示例：**

```
输入：[3,1,2,4]
输出：[2,4,3,1]
输出 [4,2,3,1]，[2,4,1,3] 和 [4,2,1,3] 也会被接受。
```

**提示：**

1. `1 <= A.length <= 5000`
2. `0 <= A[i] <= 5000`

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

