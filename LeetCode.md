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



## 排序链表

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

