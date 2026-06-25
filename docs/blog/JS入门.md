---
title: JS入门
createTime: 2026/05/13 14:01:44
tags:
    - 学习
    - 笔记
---

[参考教程：](https://liaoxuefeng.com/books/javascript/introduction/index.html)

## 简介

JavaScript是一种运行在浏览器中的解释型的编程语言。

为什么我们要学JavaScript？因为在Web世界里，只有JavaScript能跨平台、跨浏览器驱动网页，与用户交互。

为什么起名叫JavaScript？原因是当时Java语言非常红火，所以网景公司希望借Java的名气来推广，但事实上JavaScript除了语法上有点像Java，其他部分基本上没啥关系。

## 快速入门

### 基本语法

JavaScript的语法和Java语言类似，每个语句以`;`结束，语句块用`{...}`。但是，JavaScript并不强制要求在每个语句的结尾加`;`，浏览器中负责执行JavaScript代码的引擎会自动在每个语句的结尾补上`;`。

```js
var x = 1; // 赋值，不建议一行写多个。
'Hello, world'; // 字符串可以视为一个完整的语
```

### 数据类型

#### Number

JavaScript不区分整数和浮点数，统一用Number表示，以下都是合法的Number类型：

```js
123; // 整数123
0.456; // 浮点数0.456
1.2345e3; // 科学计数法表示1.2345x1000，等同于1234.5
-99; // 负数
NaN; // NaN表示Not a Number，当无法计算结果时用NaN表示
Infinity; // Infinity表示无限大，当数值超过了JavaScript的Number所能表示的最大值时，就表示为Infinity
```

Number可以直接做四则运算，规则和数学一致：

```js
1 + 2; // 3
(1 + 2) * 5 / 2; // 7.5
2 / 0; // Infinity
0 / 0; // NaN
10 % 3; // 1
10.5 % 3; // 1.5
```

JavaScript的整数最大范围不是$±2^{63}$，而是$±2^{53}$，因此，超过$2^{53}$的整数就可能无法精确表示：

```js
// 计算圆面积:
var r = 123.456;
var s = 3.14 * r * r;
console.log(s); // 47857.94555904001

// 打印Number能表示的最大整数:
console.log(Number.MAX_SAFE_INTEGER); // 9007199254740991
```

#### 字符串

字符串是以单引号'或双引号"括起来的任意文本，比如`'abc'`，`"xyz"`等等。请注意，`''`或`""`本身只是一种表示方式，不是字符串的一部分，因此，字符串`'abc'`只有`a`，`b`，`c`这3个字符。

#### 布尔值

布尔值和布尔代数的表示完全一致，一个布尔值只有`true`、`false`两种值，要么是`true`，要么是`false`，可以直接用`true`、`false`表示布尔值，也可以通过布尔运算计算出来。

#### 比较运算符

关于`==`和`===`：

- 第一种是`==`比较，它会自动转换数据类型再比较，很多时候，会得到非常诡异的结果；

- 第二种是`===`比较，它不会自动转换数据类型，如果数据类型不一致，返回`false`，如果一致，再比较。

由于JavaScript这个设计缺陷，*不要*使用`==`比较，始终坚持使用`===`比较。

```js
false == 0; // true
false === 0; // false
```

此外：

```js
NaN === NaN; // false  NaN与所有Number都不相等，包括自己
isNaN(NaN); // true  只能用isNaN判断
1 / 3 === (1 - 2 / 3); // false  精度问题
Math.abs(1 / 3 - (1 - 2 / 3)) < 0.0000001; // true 
```

#### Bight

要精确表示比253还大的整数，可以使用内置的BigInt类型，它的表示方法是在整数后加一个`n`，例如`9223372036854775808n`，也可以使用`BigInt()`把Number和字符串转换成BigInt。

使用BigInt可以正常进行加减乘除等运算，结果仍然是一个BigInt，但不能把一个BigInt和一个Number放在一起运算.

#### null和undefined

`null`表示一个“空”的值，它和`0`以及空字符串`''`不同，`0`是一个数值，`''`表示长度为0的字符串，而`null`表示“空”。

JavaScript的设计者希望用`null`表示一个空的值，而`undefined`表示值未定义。事实证明，这并没有什么卵用，区分两者的意义不大。大多数情况下，我们都应该用`null`。`undefined`仅仅在判断函数参数是否传递的情况下有用。

#### 数组

JavaScript的数组可以包括任意数据类型。

数组的元素可以通过索引来访问。请注意，索引的起始值为`0`：

```js
var arr = [1, 2, 3.14, 'Hello', null, true];
arr[0]; // 返回索引为0的元素，即1
arr[5]; // 返回索引为5的元素，即true
arr[6]; // 索引超出了范围，返回undefined
```

另一种创建数组的方法是通过`Array()`函数实现，但不建议。

#### 对象

JavaScript的对象是一组由键-值组成的无序集合，例如：

```js
var person = {
    name: 'yyx',
    age: 22,
    tags: ['js', 'web', 'mobile'],
    city: 'Nanjing',
    hasCar: true,
    zipcode: null
};
```

要获取一个对象的属性，我们用`对象变量.属性名`的方式：

```js
person.name; // 'yyx'
person.zipcode; // null
```

#### 变量

变量在JavaScript中就是用一个变量名表示，变量名是大小写英文、数字、`$`和`_`的组合，且不能用数字开头。变量名也不能是JavaScript的关键字，如`if`、`while`等。申明一个变量用`var`语句，比如：

```js
var a; // 申明了变量a，此时a的值为undefined
var $b = 1; // 申明了变量$b，同时给$b赋值，此时$b的值为1
var s_007 = '007'; // s_007是一个字符串
var Answer = true; // Answer是一个布尔值true
var t = null; // t的值是null
```

变量赋值，这种变量本身类型不固定的语言称之为动态语言：

```js
var a = 123; // a的值是整数123
a = 'ABC'; // a变为字符串
```

`let` 和 `var` 都可以用来声明变量，但现在写 JavaScript 基本上 **优先用 `let`，少用甚至不用 `var`**。

### 字符串

JavaScript的字符串就是用`''`或`""`括起来的字符表示。

如果字符串内部既包含`'`又包含`"`怎么办？可以用转义字符`\`来标识：

```js
'I\'m \"OK\"!'; // I'm "OK"!
```

转义字符`\`可以转义很多字符，比如`\n`表示换行，`\t`表示制表符，字符`\`本身也要转义，所以`\\`表示的字符就是`\`。

#### 多行字符串

由于多行字符串用`\n`写起来比较费事，所以新增了一种多行字符串的表示方法，用反引号``表示，不是单引号了：

```js
`这是一个
多行
字符串`;
```

#### 模板字符串

要把多个字符串连接起来，可以用`+`号连接。

但是如果有很多变量需要连接，用`+`号就比较麻烦。ES6新增了一种模板字符串，表示方法和上面的多行字符串一样，但是它会自动替换字符串中的变量：

```js
let name = 'yyx';
let age = 20;
let message = `你好，${name}, 今年${age}岁了`;
```

==注意，要放在``里面==

#### 操作字符串

 获取长度：

```js
let s = "111";
s.length; //3
```

要获取字符串某个指定位置的字符，使用类似Array的下标操作，索引号从0开始。

*需要特别注意的是*，字符串是不可变的，如果对字符串的某个索引赋值，不会有任何错误，但是，也没有任何效果。

#### toUpperCase 和 toLowerCase

把一个字符串全部变为大写或小写。

#### indexOf

搜索指定字符串出现的位置：

```js
let s = 'hello, world';
s.indexOf('world'); // 返回7
s.indexOf('World'); // 没有找到指定的子串，返回-1
```

#### substring

返回子串，左闭右开：

```js
let s = 'hello, world'
s.substring(0, 5); // 从索引0开始到5（不包括5），返回'hello'
s.substring(7); // 从索引7开始到结束，返回'world'
```

### 数组

#### 长度

```js
// Array.length:
let arr = [1, 2, 3.14, 'Hello', null, true];
console.log(arr.length); // 6
```

直接给`Array`的`length`赋一个新的值会导致`Array`大小的变化，多余的元素为`undefined`。

`Array`可以通过索引把对应的元素修改为新的值，因此，对`Array`的索引进行赋值会直接修改这个`Array`，如果通过索引赋值时，索引超过了范围，同样会引起`Array`大小的变化。

#### indexOF

根据元素的内容来搜索元素的索引：

```js
let arr = [10, 20, '30', 'xyz'];
arr.indexOf(10); // 元素10的索引为0
arr.indexOf(20); // 元素20的索引为1
arr.indexOf(30); // 元素30没有找到，返回-1
arr.indexOf('30'); // 元素'30'的索引为2
```

#### slice

对应String的`substring()`版本，它截取`Array`的部分元素，然后返回一个新的`Array`：

```js
let arr = ['A', 'B', 'C', 'D', 'E', 'F', 'G'];
arr.slice(0, 3); // 从索引0开始，到索引3结束，但不包括索引3: ['A', 'B', 'C']
arr.slice(3); // 从索引3开始到结束: ['D', 'E', 'F', 'G']
```

如果不给`slice()`传递任何参数，它就会从头到尾截取所有元素。利用这一点，我们可以很容易地复制一个`Array`：

```js
let arr = ['A', 'B', 'C', 'D', 'E', 'F', 'G'];
let aCopy = arr.slice();
aCopy; // ['A', 'B', 'C', 'D', 'E', 'F', 'G']
aCopy === arr; // false
```

如果`let aCopy = arr;`，那么**不是复制数组**，而是让 `aCopy` 和 `arr` 指向同一个数组。

#### push 和 pop

```js
let arr = [1, 2];
arr.push('A', 'B'); // 返回Array新的长度: 4
arr; // [1, 2, 'A', 'B']
arr.pop(); // pop()返回'B'
arr; // [1, 2, 'A']
arr.pop(); arr.pop(); arr.pop(); // 连续pop 3次
arr; // []
arr.pop(); // 空数组继续pop不会报错，而是返回undefined
arr; // []
```

#### unshift 和 shift

如果要往`Array`的头部添加若干元素，使用`unshift()`方法，`shift()`方法则把`Array`的第一个元素删掉。

#### sort

排序，可以自定义排序。

#### reverse

翻转

#### splice

`splice()`方法是修改`Array`的“万能方法”，它可以从指定的索引开始删除若干元素，然后再从该位置添加若干元素：

```js
let arr = ['Microsoft', 'Apple', 'Yahoo', 'AOL', 'Excite', 'Oracle'];
// 从索引2开始删除3个元素,然后再添加两个元素:
arr.splice(2, 3, 'Google', 'Facebook'); // 返回删除的元素 ['Yahoo', 'AOL', 'Excite']
arr; // ['Microsoft', 'Apple', 'Google', 'Facebook', 'Oracle']
// 只删除,不添加:
arr.splice(2, 2); // ['Google', 'Facebook']
arr; // ['Microsoft', 'Apple', 'Oracle']
// 只添加,不删除:
arr.splice(2, 0, 'Google', 'Facebook'); // 返回[],因为没有删除任何元素
arr; // ['Microsoft', 'Apple', 'Google', 'Facebook', 'Oracle']
```

`splice` 和`slice`的区别：

```js
slice  // 复制/截取，不改变原数组
splice // 删除/插入/替换，会改变原数组
```

#### concat

`concat()`方法把当前的`Array`和另一个`Array`连接起来，并返回一个新的`Array`：

```js
let arr = ['A', 'B', 'C'];
let added = arr.concat([1, 2, 3]);
added; // ['A', 'B', 'C', 1, 2, 3]
arr; // ['A', 'B', 'C']
```

#### join

`join()`方法是一个非常实用的方法，它把当前`Array`的每个元素都用指定的字符串连接起来，然后返回连接后的字符串：

```js
let arr = ['A', 'B', 'C', 1, 2, 3];
arr.join('-'); // 'A-B-C-1-2-3'
```

**练习题：**在新生欢迎会上，你已经拿到了新同学的名单，请排序后显示：欢迎XXX，XXX，XXX和XXX同学！：

```js
let arr = ['小明', '小红', '大军', '阿黄'];
arr = arr.sort();
console.log(`欢迎${arr.slice(0, arr.length - 1).join(', ')}和${arr.slice(arr.length - 1)}同学！`);
```

#### 多维数组

如果数组的某个元素又是一个`Array`，则可以形成多维数组，例如：

```js
let arr = [[1, 2, 3], [400, 500, 600], '-'];
```

### 对象

JavaScript的对象是一种无序的集合数据类型，它由若干键值对组成。

由于JavaScript的对象是动态类型，你可以自由地给一个对象添加或删除属性：

```js
let xiaoming = {
    name: '小明'
};
xiaoming.age; // undefined
xiaoming.age = 18; // 新增一个age属性
xiaoming.age; // 18
delete xiaoming.age; // 删除age属性
xiaoming.age; // undefined
delete xiaoming['name']; // 删除name属性
xiaoming.name; // undefined
delete xiaoming.school; // 删除一个不存在的school属性也不会报错
```

如果我们要检测`xiaoming`是否拥有某一属性，可以用`in`操作符：

```js
let xiaoming = {
    name: '小明',
    birth: 1990,
    school: 'No.1 Middle School',
    height: 1.70,
    weight: 65,
    score: null
};
'name' in xiaoming; // true
'grade' in xiaoming; // false
```

不过要小心，如果`in`判断一个属性存在，这个属性不一定是`xiaoming`的，它可能是`xiaoming`继承得到的。要判断一个属性是否是`xiaoming`自身拥有的，而不是继承得到的，可以用`hasOwnProperty()`方法：

```js
let xiaoming = {
    name: '小明'
};
xiaoming.hasOwnProperty('name'); // true
xiaoming.hasOwnProperty('toString'); // false
```

### 条件判断

略。

### 循环

#### for

略。

#### for in

`for`循环的一个变体是`for ... in`循环，它可以把一个对象的所有属性依次循环出来：

```js
let o = {
    name: 'Jack',
    age: 20,
    city: 'Beijing'
};
for (let key in o) {
    console.log(key); // 'name', 'age', 'city'
}
```

*请注意*，`for ... in`对`Array`的循环得到的是`String`而不是`Number`。

#### while 和 do ... while

略。

### Map 和 Set

#### Map

`Map`是一组键值对的结构，具有极快的查找速度。

```js
let m = new Map([['Michael', 95], ['Bob', 75], ['Tracy', 85]]);
m.get('Michael'); // 95
```

用法：

```js
let m = new Map(); // 空Map
m.set('Adam', 67); // 添加新的key-value
m.set('Bob', 59);
m.has('Adam'); // 是否存在key 'Adam': true
m.get('Adam'); // 67
m.delete('Adam'); // 删除key 'Adam'
m.get('Adam'); // undefined
```

会覆盖：

```js
let m = new Map();
m.set('Adam', 67);
m.set('Adam', 88);
m.get('Adam'); // 88
```

#### Set

`Set`和`Map`类似，也是一组key的集合，但不存储value。由于key不能重复，所以，在`Set`中，没有重复的key。

```js
let s1 = new Set(); // 空Set
let s2 = new Set([1, 2, 3]); // 含1, 2, 3
```

add和delete，add的重复元素会自动过滤：

```js
let s = new Set([1, 2, 3, 3, '3']);
s; // Set {1, 2, 3, "3"}
s.add(4);
s; // Set {1, 2, 3, 4}
s.add(4);
s; // 仍然是 Set {1, 2, 3, 4}

let s = new Set([1, 2, 3]);
s; // Set {1, 2, 3}
s.delete(3);
s; // Set {1, 2}

```

### iterable

遍历`Array`可以采用下标循环，遍历`Map`和`Set`就无法使用下标。为了统一集合类型，ES6标准引入了新的`iterable`类型，`Array`、`Map`和`Set`都属于`iterable`类型。

#### for of

具有`iterable`类型的集合可以通过新的`for ... of`循环来遍历。

```js
let a = ['A', 'B', 'C'];
let s = new Set(['A', 'B', 'C']);
let m = new Map([[1, 'x'], [2, 'y'], [3, 'z']]);
for (let x of a) { // 遍历Array
    console.log(x);
}
for (let x of s) { // 遍历Set
    console.log(x);
}
for (let x of m) { // 遍历Map
    console.log(x[0] + '=' + x[1]);
}
```

`for ... of`循环和`for ... in`循环有何区别？

==`for ... in`循环由于历史遗留问题，它遍历的实际上是对象的属性名称。一个`Array`数组实际上也是一个对象，它的每个元素的索引被视为一个属性。==

#### forEach

更好的方式是直接使用`iterable`内置的`forEach`方法，它接收一个函数，每次迭代就自动回调该函数。以`Array`为例：

```js
let a = ['A', 'B', 'C'];
a.forEach(function (element, index, array) {
    // element: 指向当前元素的值
    // index: 指向当前索引
    // array: 指向Array对象本身
    console.log(`${element}, index = ${index}`);
});
```

`Set`与`Array`类似，但`Set`没有索引，因此回调函数的前两个参数都是元素本身（这是为了让 `Set.forEach()` 的参数形式和 `Array.forEach()`、`Map.forEach()` 保持相似）：

```js
let s = new Set(['A', 'B', 'C']);
s.forEach(function (element, sameElement, set) {
    console.log(element);
  	// 等同于 console.log(sameElement);
});
```

==这里的`element`和`sameElement`没有区别，只是名字不一样==

`Map`的回调函数参数依次为`value`、`key`和`map`本身：

```js
let m = new Map([[1, 'x'], [2, 'y'], [3, 'z']]);
m.forEach(function (value, key, map) {
    console.log(value);
});
```

如果对某些参数不感兴趣，由于JavaScript的函数调用不要求参数必须一致，因此可以忽略它们。例如，只需要获得`Array`的`element`：

```js
let a = ['A', 'B', 'C'];
a.forEach(function (element) {
    console.log(element);
});
```

## 函数

### 函数定义和调用

#### 定义

**第一种定义方式：**

```js
function abs(x) {
    if (x >= 0) {
        return x;
    } else {
        return -x;
    }
}
```

如果没有`return`语句，函数执行完毕后也会返回结果，只是结果为`undefined`。

**第二种定义方式:**

```js
let abs = function (x) {
    if (x >= 0) {
        return x;
    } else {
        return -x;
    }
};
```

在这种方式下，`function (x) { ... }`是一个匿名函数，它没有函数名。但是，这个匿名函数赋值给了变量`abs`，所以，通过变量`abs`就可以调用该函数。

上述两种定义*完全等价*，注意第二种方式按照完整语法需要在函数体末尾加一个`;`，表示赋值语句结束。

#### 调用函数

JavaScript函数允许接收任意个参数。

```js
function abs(x) {
    if (typeof x !== 'number') {
        throw 'Not a number';
    }
    if (x >= 0) {
        return x;
    } else {
        return -x;
    }
}
```

#### arguments

JavaScript还有一个免费赠送的关键字`arguments`，它只在函数内部起作用，并且永远指向当前函数的调用者传入的所有参数。`arguments`类似`Array`但它不是一个`Array`：

```js
function abs() {
    if (arguments.length === 0) {
        return 0;
    }
    let x = arguments[0];
    return x >= 0 ? x : -x;
}

abs(); // 0
abs(10); // 10
abs(-9); // 9
```

`arguments`最常用于判断传入参数的个数。

#### rest参数

rest可以获取额外的参数，rest参数只能写在最后，前面用`...`标识。

```js
function foo(a, b, ...rest) {
    console.log('a = ' + a);
    console.log('b = ' + b);
    console.log(rest);
}

foo(1, 2, 3, 4, 5);
// 结果:
// a = 1
// b = 2
// Array [ 3, 4, 5 ]

foo(1);
// 结果:
// a = 1
// b = undefined
// Array []
```

### 变量作用域与解构赋值

#### 全局作用域

不在任何函数内定义的变量就具有全局作用域。实际上，JavaScript默认有一个全局对象`window`，全局作用域的变量实际上被绑定到`window`的一个属性：

```js
var course = 'Learn JavaScript';
console.log(course); // 'Learn JavaScript'
console.log(window.course); // 'Learn JavaScript'
```

#### 解构赋值

从ES6开始，JavaScript引入了解构赋值，可以同时对一组变量进行赋值。

```js
let array = ['hello', 'JavaScript', 'ES6'];
let x = array[0];
let y = array[1];
let z = array[2];
```

还可以忽略某些元素：

```js
let [, , z] = ['hello', 'JavaScript', 'ES6']; // 忽略前两个元素，只对z赋值第三个元素
z; // 'ES6'
```

解构赋值还可以使用默认值，这样就避免了不存在的属性返回`undefined`的问题：

```js
let person = {
    name: '小明',
    age: 20,
    gender: 'male',
    passport: 'G-12345678'
};

// 如果person对象没有single属性，默认赋值为true:
let {name, single=true} = person;
name; // '小明'
single; // true
```

### 方法

```js
let xiaoming = {
    name: '小明',
    birth: 1990,
    age: function () {
        let y = new Date().getFullYear();
        return y - this.birth;
    }
};

xiaoming.age; // function xiaoming.age()
xiaoming.age(); // 今年调用是25,明年调用就变成26了
```

### 高阶函数

JavaScript的函数其实都指向某个变量。既然变量可以指向函数，函数的参数能接收变量，那么一个函数就可以接收另一个函数作为参数，这种函数就称之为高阶函数。

```js
function add(x, y, f) {
    return f(x) + f(y);
}
```

#### map

由于`map()`方法定义在JavaScript的`Array`中，我们调用`Array`的`map()`方法，传入我们自己的函数，就得到了一个新的`Array`作为结果：

```js
function pow(x) {
    return x * x;
}

let arr = [1, 2, 3, 4, 5, 6, 7, 8, 9];
let results = arr.map(pow); // [1, 4, 9, 16, 25, 36, 49, 64, 81]
console.log(results);
```

数组数字转字符：

```js
let arr = [1, 2, 3, 4, 5, 6, 7, 8, 9];
arr.map(String); // ['1', '2', '3', '4', '5', '6', '7', '8', '9']
```

#### reduce

Array的`reduce()`把一个函数作用在这个`Array`的`[x1, x2, x3...]`上，这个函数必须接收两个参数，`reduce()`把结果继续和序列的下一个元素做累积计算，其效果就是：

```js
[x1, x2, x3, x4].reduce(f) = f(f(f(x1, x2), x3), x4)
```

对一个数组求和：

```js
let arr = [1, 3, 5, 7, 9];
arr.reduce(function (x, y) {
    return x + y;
}); // 25
```

**练习题1：**想办法把一个字符串`13579`先变成`Array`——`[1, 3, 5, 7, 9]`，再利用`reduce()`就可以写出一个把字符串转换为Number的函数。

```js
function string2int(s) {
    // FIXME:
  	return s.split('').map(x => x - '0').reduce((x,y) => x *10 + y);
}

// 测试:
if (string2int('0') === 0 && string2int('12345') === 12345 && string2int('12300') === 12300) {
    if (string2int.toString().indexOf('parseInt') !== -1) {
        console.log('请勿使用parseInt()!');
    } else if (string2int.toString().indexOf('Number') !== -1) {
        console.log('请勿使用Number()!');
    } else {
        console.log('测试通过!');
    }
}
else {
    console.log('测试失败!');
}
```

**练习题2：**请把用户输入的不规范的英文名字，变为首字母大写，其他小写的规范名字。输入：`['adam', 'LISA', 'barT']`，输出：`['Adam', 'Lisa', 'Bart']`。

```js
function normalize(arr) {
    // FIXME:
  	function f(s) {
      let [first, ...rest] = s;
      return first.toUpperCase() + rest.join('').toLowerCase();
    }
  	return arr.map(f);
}
//或者这么写
function normalize(arr) {
    // FIXME:
  	function f(s) {
      return s[0].toUpperCase() + s.slice(1).toLowerCase();
    }
  	return arr.map(f);
}


// 测试:
if (normalize(['adam', 'LISA', 'barT']).toString() === ['Adam', 'Lisa', 'Bart'].toString()) {
    console.log('测试通过!');
}
else {
    console.log('测试失败!');
}
```

#### filter

filter也是一个常用的操作，它用于把`Array`的某些元素过滤掉，然后返回剩下的元素。和`map()`类似，`Array`的`filter()`也接收一个函数。和`map()`不同的是，`filter()`把传入的函数依次作用于每个元素，然后根据返回值是`true`还是`false`决定保留还是丢弃该元素。

只保留数组里面的奇数：

```js
let arr = [1, 2, 4, 5, 6, 9, 10, 15];
let r = arr.filter(function (x) {
    return x % 2 !== 0;
});
r; // [1, 5, 9, 15]
```

`filter()`接收的回调函数，其实可以有多个参数。通常我们仅使用第一个参数，表示`Array`的某个元素。回调函数还可以接收另外两个参数，表示元素的位置和数组本身：

```js
let arr = ['A', 'B', 'C'];
let r = arr.filter(function (element, index, self) {
    console.log(element); // 依次打印'A', 'B', 'C'
    console.log(index); // 依次打印0, 1, 2
    console.log(self); // self就是变量arr
    return true;
});
```

#### sort

倒序排序：

```js
let arr = [10, 20, 1, 2];
arr.sort(function (x, y) {
    return y - x;
}); // [20, 10, 2, 1]
```

#### every

`every()`方法可以判断数组的**所有元素**是否满足测试条件。

```js
let arr = ['Apple', 'pear', 'orange'];
console.log(arr.every(function (s) {
    return s.length > 0;
})); // true, 因为每个元素都满足s.length>0

console.log(arr.every(function (s) {
    return s.toLowerCase() === s;
})); // false, 因为不是每个元素都全部是小写
```

#### find/findIndex

`find()`方法用于查找符合条件的第一个元素，如果找到了，返回这个元素，否则，返回`undefined`。

`findIndex()`和`find()`类似，也是查找符合条件的第一个元素，不同之处在于`findIndex()`会返回这个元素的索引，如果没有找到，返回`-1`。

#### forEach

`forEach()`和`map()`类似，它也把每个元素依次作用于传入的函数，但不会返回新的数组。`forEach()`常用于遍历数组，因此，传入的函数不需要返回值：

```js
let arr = ['Apple', 'pear', 'orange'];
arr.forEach(x=>console.log(x)); // 依次打印每个元素
```

### 闭包

我直接跳过了。

### 箭头函数

```js
let arr = [10, 20, 1, 2];
arr.sort((x, y) => x - y);
console.log(arr); // [1, 2, 10, 20]
```

### 标签函数

跳过。

#### 生成器

函数在执行过程中，如果没有遇到`return`语句（函数末尾如果没有`return`，就是隐含的`return undefined;`），控制权无法交回被调用的代码。

但是，generator和函数不同的是，generator由`function*`定义（注意多出的`*`号），并且，除了`return`语句，还可以用`yield`返回多次。

举例（斐波那契数列的函数）：

```js
function* fib(max) {
    let
        t,
        a = 0,
        b = 1,
        n = 0;
    while (n < max) {
        yield a;  // 中途返回，但不停止函数
        [a, b] = [b, a + b];
        n ++;
    }
    return; // 到这里才结束函数
}
```

**如何调用生成器？**

一种是不断地调用generator对象的`next()`方法：

```js
let f = fib(5);
f.next(); // {value: 0, done: false}
f.next(); // {value: 1, done: false}
f.next(); // {value: 1, done: false}
f.next(); // {value: 2, done: false}
f.next(); // {value: 3, done: false}
f.next(); // {value: undefined, done: true}
```

二是直接用`for ... of`循环迭代generator对象，这种方式不需要我们自己判断`done`：

```js
for (let x of fib(10)) {
    console.log(x); // 依次输出0, 1, 1, 2, 3, ...
}
```

小例子： 

```js
function* next_id() {
    let x = 0;
  	while(1){
      yield ++x;
    }
}

// 测试:
let
    x,
    pass = true,
    g = next_id();
for (x = 1; x < 100; x ++) {
    if (g.next().value !== x) {
        pass = false;
        console.log('测试失败!');
        break;
    }
}
if (pass) {
    console.log('测试通过!');
}
```

## 标准对象

### Date

```js
let d = new Date(2015, 5, 19, 20, 15, 30, 123);
console.log(d); // Fri Jun 19 2015 20:15:30 GMT+0800 (CST)
```

一个*非常非常坑爹*的地方，就是JavaScript的月份范围用整数表示是0~11，`0`表示一月，`1`表示二月……，所以要表示6月，我们传入的是`5`！这绝对是JavaScript的设计者当时脑抽了一下，但是现在要修复已经不可能了。
