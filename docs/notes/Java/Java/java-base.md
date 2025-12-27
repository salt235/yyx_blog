---
title: java-base
createTime: 2025/12/27 14:44:18
permalink: /notes/Java/xisqkup4/
---


## IDEA快捷键

- ctrl + D 复制当前行到下一行

- alt + shift + ↑ 上下移动当前行

- ctrl + / 注释单行

- 选择一段按 ctrl + alt  + T 可以在外层添加if while等等

- ctrl + shift + / 多行注释

- alt + enter 帮你强制转换类型 

- 要遍历a，输入a.fori然后按回车


## 语法知识

- long类型后面要加L不然默认int

- float类型后面要加F不然默认double

- 写小数默认是double类型

- 类型转换的优先级 byte -> short -> int -> long -> float -> double

- float强制转换为int只取整数部分

- 逻辑运算符 & | ! ^，^是异或

- &&短路与 ||短路或，和c++一样了，左边判断了可以得到结果就不看右边了


## 其他知识

- API(Application Programming Interface)：应用程序编程接口


## 面向对象知识

- 一个代码文件中，可以有多个class，但只能有一个public修饰，却public修饰的要和文件名一样

- 一旦手写了有参构造函数，那么必须也要手写一个无参构造函数，因为它不会自动生成了


## String

- **构建String**

```java
// 字符数组转换为字符串
char[] chars = {'y', 'y', 'x'};
String rs1 = new String(chars);
System.out.println(rs1);

// 字节数组转换为字符串
byte[] bytes = {97, 98, 99};
String rs2 = new String(bytes);
System.out.println(rs2);
```

- **String常用方法**

```java
// 1.长度
System.out.println(rs1.length());

// 2.索引，先输入rs1.length().fori按回车，charAt索引
for (int i = 0; i < rs1.length(); i++) {
    System.out.println(rs1.charAt(i));
}

// 3.toCharArray，字符串转换为字符数组
char[] chs = rs1.toCharArray();
for (int i = 0; i < chs.length; i++) {
    System.out.println(chs[i]);
}

// 4.字符串是否相等。s1s2与s3s4的构造方式不一样，细节
// 只要是以“”方式写出的字符串对象，会存储到字符串常量池，且相同字符只储存一份（地址一样）
// 但是以new方式写出的，每new一次会产生一个新地址
String s1 = "123";
String s2 = "123";
String s3 = new String("234");
String s4 = new String("234");
System.out.println(s1 == s2); // true，这个比较的是值
System.out.println(s3 == s4); // false，这个比较的是地址
System.out.println(s1.equals(s2)); // true
System.out.println(s3.equals(s4)); // true\

// 5.忽略大小写比较
String s5 = "yyxx";
String s6 = "YyXx";
System.out.println(s5.equalsIgnoreCase(s6)); // true

// 6.截取字符串内容substring
String s7 = "34截取df";
System.out.println(s7.substring(0, 4));// 两个参数就是头到尾 "34截取"
System.out.println(s7.substring(4));// 一个参数就是起始位置到最后 "df"

// 7.字符串某内容替换为新内容replace
String info = "你是个shabi";
info = info.replace("shabi","niuj");
System.out.println(info); // 你是个niuj

// 8.找关键词contains
String ss1 = "我爱java，鸡脖";
System.out.println(ss1.contains("java")); // true

// 10.判断字符串是否以某个开头startwith
System.out.println(ss1.startsWith("我爱"));// true

// 11.分割字符串到几个字符串数组里split
String ss2 = "蔡徐坤，陈立农，丁晨曦，宋亚轩";
String names[] = ss2.split("，"); // 以逗号隔开
for (int i = 0; i < names.length; i++) {
    System.out.println(names[i]);
}
```

## ArrayList(集合/容器)

```java
// 创建容器
ArrayList list = new ArrayList();

// 1.添加元素到末尾
list.add(12);
list.add("232");
System.out.println(list); // [12, 232]

// 2.添加特定种类的元素
ArrayList<String> list2 = new ArrayList<>();
list2.add("343");
list2.add("dfdf");

// 3.特定位置插入，插入到某个位置的前面
list.add(1,"插入");
System.out.println(list);

// 4.get获取某个索引位置的值
String rs = list2.get(1);
System.out.println(rs); // dfdf

// 5.集合大小size
int a = list.size();

// 6.删除指定索引，remove
list.remove(1);
System.out.println(list); // [12, 232]

// 7.直接删除关键词，有相同的话删除第一个出现的，remove
list2.remove("dfdf");
System.out.println(list2); // [343]

// 8.修改某个索引的位置set
list.set(1, "ono");
System.out.println(list);
```

