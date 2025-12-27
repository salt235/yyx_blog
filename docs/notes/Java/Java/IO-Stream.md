---
title: IO-Stream
createTime: 2025/12/27 14:44:18
permalink: /notes/Java/frgoemdz/
---
# IO流

[TOC]



## File

- 绝对路径 vs 相对路径

 （1）绝对路径：是一个固定的路径,从盘符开始
 （2）相对路径：是相对于某个位置开始

```java
// 绝对路径
File f = new File("F:/file/六百六十六.txt");
System.out.println(f.length()); // 文件大小
System.out.println(f.exists()); // 是否存在
// 相对路径
File f2 = new File("666.txt");
System.out.println(f2.exists());
```
- **常用方法1（获取文件各种信息）**

1. exists()：是否存在
2. isFile()：是否是文件
3. isDirectory()：是否是文件夹
4. getName()：获取文件名称
5. length()：获取文件大小（字节数）
6. lastModified()：最后修改时间，返回类型是long。

```java
File f = new File("F:/file/六百六十六.txt");
long time = f.lastModified();
SimpleDateFormat sdf = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
System.out.println(sdf.format(time));
```

7. getPath()：获取文件创建时的路径

8. getAbsolutePath()：获取绝对路径

- **常用方法2（创建删除）**

1. 创建文件：createNewFile()
 ```java
 // 如果文件已存在则返回false
 File f1 = new File("F:/file/七百七十七.txt");
 System.out.println(f1.createNewFile());
 ```
2. 创建一级文件夹：mkdir()
>在已知目录下面最多再创一个文件夹
>
```java
File f2 = new File("F:/file/t1");
System.out.println(f2.mkdir());
```

3. 创建多级文件夹：mkdirs()
>在已知目录下面可以创多个文件夹
```java
File f3 = new File("F:/file/t2/tt2/ttt2");
System.out.println(f3.mkdirs());
```
4. 删除文件或者空文件（无法删除非空文件夹）：delete()

- **常用方法3（遍历）**

1. 获取当前文件夹下面的一级文件

```java
File f = new File("F:/file");
String[] names = f.list();
System.out.println(Arrays.toString(names));
```

2. 获取当前目录下所有的”一级文件对象“到一个文件对象数组中

```java
File f = new File("F:/Java-Note");
File[] files = f.listFiles();
for (File file : files) {
    System.out.println(file.getAbsoluteFile());
}
```

## 遍历搜索文件

```java
// 传入根目录和要搜索的文件名
public static void searchFile(File dir, String fileName){
    // 判断dir是否合法
    if(dir == null || !dir.exists() || dir.isFile())
        return;

    File[] files = dir.listFiles();
	// 跳过空文件
    if(files != null && files.length > 0){
        for (File file : files) {
            if(file.isFile() && file.getName().contains(fileName)){
                    System.out.println(file.getAbsolutePath());
            } else {
                searchFile(file, fileName); // 递归
            }
        }
    }
}
```

## 删除非空文件夹

```java
public static void deleteDir(File dir){
    if(dir == null || !dir.exists())
        return ;

    // 如果是文件则删除
    if(dir.isFile())
        dir.delete();

    // 如果是文件夹
    File[] files = dir.listFiles();
    // 确保为文件夹非空，重点，不写这个if会报错
    if (files != null) {
        for (File file : files) {
            deleteDir(file);
        }
    }
    // 最后清除本身
    dir.delete();
}
```

## 字符集

- ASCII字符集：只有英文、数字、符号等，占1个字节。

- GBK字符集：汉字占2个字节，英文、数字占1个字节。

- UTF-8字符集：汉字占3个字节，英文、数字占1个字节。

==字符编码与解码时使用的字符集必须一致。==

==英文数字一般不会乱码，因为很多字符集兼容了ASCII编码。==

- 编码

```java
String data = "a鹅b";
// 按照默认字符集编码，输出每个字节
byte[] bytes = data.getBytes();
System.out.println(Arrays.toString(bytes)); // [97, -23, -71, -123, 98]
// 按照指定字符集编码
byte[] bytes1 = data.getBytes("GBK");
System.out.println(Arrays.toString(bytes1)); // [97, -74, -20, 98]
```

- 解码

```java
// 按照默认字符集解码，输出内容
String s1 = new String(bytes);
System.out.println(s1); // a鹅b
// 按照指定字符集解码
String s2 = new String(bytes1, "GBK");
System.out.println(s2); // a鹅b
```

## IO流

- 分类

 **按数据流向分：**

输入流： 用于从外部读取数据到程序中。

输出流： 用于将程序中的数据写入到外部。

**按处理数据单位分：**

字节流：以字节为单位处理数据，适用于所有类型的I/O操作，包括二进制文件（如图片、音频等）。

字符流：以字符为单位处理数据，适用于文本文件的I/O操作。

------



## FileInputStream（文件字节输入流）

> 把磁盘文件中的数据以字节的形式读入到内存。

- 一次读取一个字节，读取汉字会乱码！

```java
// 创建文件字节输入流管道
InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");
// 读取文件的每一个字节，读取的性能很差
int b;
while((b = is.read()) != -1){
    System.out.print((char) b);
}
// 流用完后要释放
is.close();
```
- 一次读取多个字节，读取汉字也会乱码！

```java
// 创建文件字节输入流管道
InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");

// 一次性读取多个字节
byte[] buffer = new byte[3];
int len; // 记录每次多了几个字节
while((len = is.read(buffer)) != -1){
    String rs = new String(buffer, 0, len);
    System.out.print(rs);
}
is.close();
```

- 一次读取整个文件，读取汉字不会乱码！

```java
// 创建文件字节输入流管道
InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");

// 一次性读取文件的全部
File f = new File("iostream\\src\\IO\\1.txt");
byte[] buffer = new byte[(int)f.length()];
int len = is.read(buffer);
String rs = new String(buffer);
System.out.print(rs);

is.close();
```

- 用readAllBytes()来读取整个文件（以上经典白雪）

```java
// 创建文件字节输入流管道
InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");

byte[] buffer = is.readAllBytes();
System.out.print(new String(buffer));

is.close();
```

## FileOutputStream（文件字节输出流）

不记笔记了，白雪警告。

- 文件的拷贝

```java
InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");
OutputStream os = new FileOutputStream("iostream\\src\\IO\\2.txt");

byte[] buffer = new byte[1024];

int len;
while((len = is.read(buffer)) != -1){
    os.write(buffer, 0, len);
}
is.close();
os.close();
```

## 释放资源

==可以用try-catch-finally来释放==，但是臃肿。

- try-with-resourse

```java
try (
        // 这里只能放置资源对象
        InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");
        OutputStream os = new FileOutputStream("iostream\\src\\IO\\2.txt");
        ){
    byte[] buffer = new byte[1024];
    int len;
    while((len = is.read(buffer)) != -1){
        os.write(buffer, 0, len);
    }

} catch (IOException e) {
    throw new RuntimeException(e);
}
```

## FileReader（文件字符输入流）
- 一次读取一个字符
```java
Reader fr = new FileReader("iostream\\src\\IO\\1.txt")
// 记住每次读取的字符
int c;
while((c = fr.read()) != -1){
    System.out.print((char) c);
}
```

- 一次读取多个字符

```java
Reader fr = new FileReader("iostream\\src\\IO\\1.txt")
char[] buffer = new char[3];
int len;
while((len = fr.read(buffer)) != -1){
    System.out.print(new String(buffer, 0, len));
}
```

## FileWriter（文件字符输出流）

```java
Writer fw = new FileWriter("iostream\\src\\IO\\3.txt", true);
fw.write("666");
fw.write("\r\n");// 换行
fw.write("666");
// write里面可以放字符、字符串、字符数组，也可以加上偏移量只写入字符串或者字符数组的一部分
fw.close();
```

==字符输出流写出数据后，必须刷新流或者关闭流，写出去的数据才能生效。==

刷新流fw.flush();

## 缓冲流
- 字节缓冲流
```java
try (
    	// 用BufferedInputStream和BufferedOutputStream来包装原来的低级流
        InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");
        InputStream bis = new BufferedInputStream(is);
        OutputStream os = new FileOutputStream("iostream\\src\\IO\\2.txt");
        OutputStream bos = new BufferedOutputStream(os);
        ){
    byte[] buffer = new byte[1024];

    int len;
    while((len = bis.read(buffer)) != -1){
        bos.write(buffer, 0, len);
    }
} catch (IOException e) {
    throw new RuntimeException(e);
}
```

- 字符缓冲流

同上，但是

字符缓冲输入流多了一个readLine()功能来读取一行，返回值是String。

字符串流输出流多了一个newLine()功能来换行，返回值是void。

## 转换流
- 字符输入转换流InputStreamReader

```java
try (
        // 得到原始字节流，GBK编码的
        InputStream is = new FileInputStream("iostream\\src\\IO\\1.txt");
        // 字节流按照特定字符集编码转换为字符流
        Reader isr = new InputStreamReader(is, "GBK");
        // 把字符流包装为缓冲流
        BufferedReader br = new BufferedReader(isr);
        ){
    // 逐行读取
    String line;
    while ((line = br.readLine()) != null){
        System.out.println(line);
    }
} catch (Exception e) {
    e.printStackTrace();
}
```

- 字符输出转换流OutputStreamWriter

## 打印流（PrintStream/PrintWriter）

```java
try (
        PrintStream ps = new PrintStream("iostream\\src\\IO\\4.txt", Charset.forName("UTF-8"));
        ){
        ps.println(97);
        ps.println("dfdf酒店");
        ps.write(97); // 写入一个字符'a'
        
} catch (Exception e) {
    e.printStackTrace();
}
```

如果想要能追加数据，要用高级流包装低级流

```java
PrintStream ps = new PrintStream(new FileOutputStream("iostream\\src\\IO\\4.txt", true));
```

- 打印流的应用（输出语句的重定向）

```java
try (PrintStream ps = new PrintStream("iostream\\\\src\\\\IO\\\\5.txt");){

    // 把系统默认的的打印流对象改成自己设置的打印流
    System.setOut(ps);
    System.out.println("六百六十六");
    System.out.println("111");

} catch (Exception e) {
    e.printStackTrace();
}
```

## 数据流

- 数据输出流DataOutputStream

```java
try (
    // 用高级流包装低级流
        DataOutputStream dos = new DataOutputStream(new FileOutputStream("F:iostream\\src\\IO\\5.txt"));
        ){

    // 写入文件可以一并将类型写进去
    dos.writeInt(343);
    dos.writeBoolean(true);
    dos.writeDouble(1.1);
    dos.writeUTF("dfdfdf");
    
} catch (Exception e) {
    e.printStackTrace();
}
```

- 数据输入流DataInputStream

```java
try (
        DataInputStream dis = new DataInputStream(new FileInputStream("F:iostream\\src\\IO\\5.txt"));
        ){
    // 必须按照文件中数据类型的先后顺序读
    System.out.println(dis.readInt());
    System.out.println(dis.readBoolean());
    System.out.println(dis.readDouble());
    System.out.println(dis.readUTF());

} catch (Exception e) {
    e.printStackTrace();
}
```

## 序列化流

> 把java对象写入文件或者读取出来

- 对象字节输出流ObjectOutputStream

```java
// 对象如果需要序列化，必须实现序列化接口
public class Stu implements Serializable
```

```java
// 创建一个对象字节输出流包装原始的字节输出流
try (
        ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream("iostream\\src\\IO\\6.txt"));
        ){

    // 创建一个java对象
    Stu s = new Stu("yyx", 100.0);
    // 序列化对象到文件中
    oos.writeObject(s);
    
} catch (Exception e) {
    e.printStackTrace();
}
```

- 对象字节输入流ObjectInputStream

```java
try (
        ObjectInputStream ois = new ObjectInputStream(new FileInputStream("iostream\\src\\IO\\6.txt"));
        ){

    Stu s = (Stu)ois.readObject();
    System.out.println(s);

} catch (Exception e) {
    e.printStackTrace();
}
```

==ps：如果在序列化之后重写了toString方法再反序列化会报错，需要重新运行序列化再反序列化，因为存进去的没有toString，拿出来的有toStirng。==
