[TOC]

# LeetCode(Second month)

## 简单

### 两个数组的交集II

```python
class Solution(object):
    def intersect(self, nums1, nums2):
        """
        :type nums1: List[int]
        :type nums2: List[int]
        :rtype: List[int]
        """
#         if not nums1 or not nums2:
#             return []
#         ret = []
        
#         for i in nums1:
#             if i in nums2:
#                 if i not in ret: 
#                     ret += [i] * (nums1.count(i) if nums1.count(i)<=nums2.count(i) else nums2.count(i))
#         return ret
        
        m = len(nums1)
        n = len(nums2)

        if m == 0 or n == 0:
            return []


        dicts = {}
        for i in nums1:
            if i in dicts:
                dicts[i] += 1
            else:
                dicts[i] = 1

        ret = []
        for j in nums2:
            if j in dicts and dicts[j] > 0:
                ret.append(j)
                dicts[j] -= 1

        return ret
```

### 加一

```python
class Solution(object):
    def plusOne(self, digits):
        """
        :type digits: List[int]
        :rtype: List[int]
        """
        carry = 1
        for i in xrange(len(digits)-1,-1,-1):
            if carry == 0:
                return digits
            tmp = digits[i] + carry
            carry = tmp/10
            digits[i] = tmp%10
        
        if carry != 0:
            ret = [1]+digits
            return ret
        return digits
```

### 有效的字母异位词

```python
class Solution(object):
    def isAnagram(self, s, t):
        """
        :type s: str
        :type t: str
        :rtype: bool
        """
        ls = len(s)
        lt = len(t)
        if ls != lt:
            return False
        c = set(t)
        for i in c:
            if t.count(i) != s.count(i):
                return False
        return True
```

### 旋转数组

```python
class Solution(object):
    def rotate(self, nums, k):
        """
        :type nums: List[int]
        :type k: int
        :rtype: void Do not return anything, modify nums in-place instead.
        """
        
        l = len(nums)
        k = k%l
        if not nums or k == 0:
            return None
        
#         count = 0
#         i = 0
#         start = 0
#         cur = nums[i]
#         while count < l:
#             i = (i+k)%l
#             tmp = nums[i]
#             nums[i] = cur
#             if i == start:
#                 start += 1
#                 i += 1
#                 cur = nums[i]
#             else:
#                 cur = tmp
#             count += 1

        nums[:l-k] = nums[:l-k][::-1]
        nums[l-k:] = nums[l-k:][::-1]
        nums[:] = nums[:][::-1]
        
        # nums[:k],nums[k:]=nums[n-k:],nums[:n-k]
```

### 验证回文字符串

```python
class Solution(object):
    def isPalindrome(self, s):
        """
        :type s: str
        :rtype: bool
        """
#         if not s:
#             return True
#         s = s.lower()
#         st = ''
#         for i in s:
#             if 'a'<= i <= 'z' or '0' <= i <= '9':
#                 st += i
                
#         return st == st[::-1]
        import re
        a = ''.join(re.findall('[0-9a-zA-Z]+',s)).upper()
        return a == a[::-1]
```

### 实现strStr()

```python
class Solution(object):
    def strStr(self, haystack, needle):
        """
        :type haystack: str
        :type needle: str
        :rtype: int
        """
        if not needle:
            return 0
        if not (needle in haystack):
            return -1
        # return haystack.find(needle)
        l = len(needle)
        for i in range(len(haystack) - l + 1):
            if haystack[i:i+l] == needle:
                return i
```

### 计数质数

```python
class Solution(object):
    def countPrimes(self, n):
        """
        :type n: int
        :rtype: int
        """
        if n < 3:
            return 0
        flag = [1]*(n+1)
        count = 0
        for i in xrange(2,n):
            if flag[i] == 1:
                j = i+i
                while j<n:
                    flag[j] = 0
                    j += i
                count += 1
        return count
```

### 对称二叉树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def isSymmetric(self, root):
        """
        :type root: TreeNode
        :rtype: bool
        """
        # BFS 广度优先搜索
        
        # if not root:
        #     return True
        # nodeleft = [root.left]
        # noderight = [root.right]
        # while nodeleft:
        #     nodeleftnext = []
        #     noderightnext = []
        #     for i in range(len(nodeleft)):
        #         if nodeleft[i] and noderight[i]:
        #             if nodeleft[i].val != noderight[i].val:
        #                 return False
        #             else:
        #                 nodeleftnext.append(nodeleft[i].left)
        #                 nodeleftnext.append(nodeleft[i].right)
        #                 noderightnext.append(noderight[i].right)
        #                 noderightnext.append(noderight[i].left)
        #         elif nodeleft[i] or noderight[i]:
        #             return False
        #     nodeleft = nodeleftnext
        #     noderight = noderightnext
        # return True
        
        #DFS 深度优先搜索
        
        if not root:
            return True
        return self.dfs(root.left, root.right)
    def dfs(self, p, q):
        if not p and not q:
            return True
        elif p and q:
            if p.val != q.val:
                return False
            return self.dfs(p.left, q.right) and self.dfs(p.right, q.left)
        elif p or q:
            return False
```

### 卡牌分组

```python
class Solution(object):
    def hasGroupsSizeX(self, deck):
        """
        :type deck: List[int]
        :rtype: bool
        """
        lt = list(set(deck))
        stack = []
        for i in lt:
            stack.append(deck.count(i))
        if min(stack) <2:
            return False
        g = -1
        for i in stack:
            if g<0:
                g = i
            else:
                g = self.gcb(i,g)
            
        return g > 1
    
    def gcb(self, a, b):
        if a<b:
            tmp = b
            b = a
            a = tmp
        while b>0:
            c = a
            a = b
            b = c%b
        return a
```

### 字符串中的第一个唯一字符

```python
class Solution(object):
    def firstUniqChar(self, s):
        """
        :type s: str
        :rtype: int
        """
        if not s:
            return -1
        lt = list(set(s))
        lt.sort(key=s.index)
        print lt
        for i in range(len(lt)):
            if s.count(lt[i]) == 1:
                print lt[i]
                return s.find(lt[i])
        return -1
```

## 中等

### 验证二叉搜索树

```python
# Definition for a binary tree node.
# class TreeNode(object):
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution(object):
    def isValidBST(self, root):
        """
        :type root: TreeNode
        :rtype: bool
        """
        if not root:
            return True
        print 2**31
        return self.isBST(root, -2**31-1, 2**31)
    def isBST(self, root, mn, mx):
        if not root:
            return True
        elif not (mn<root.val<mx):
            return False
        else:
            return self.isBST(root.left, mn, root.val) and self.isBST(root.right, root.val, mx)
```

