### numel
return number of elements in an array or subscripted array expression.
N = numel(A) returns the number of elements, N, in array A, equivalent 
to PROD(SIZE(A))

#### eg:<br>
```matlab
>> tmp = ones(10,1);
>> numel(tmp)
ans =
    10
```

### 转置

#### 共轭转置
```
conj(A) or A' 
```

#### 非共轭转置
```
transpose(A) or A.'
```

```matlab
a =

        12.0000                  0 + 2.0000i         5.0000          
        0                             5.0000               4.0000 

>> a'

             ans =

                      12.0000                  0          
                      0 - 2.0000i              5.0000          
                      5.0000                    4.0000         

>> a.'

           ans =

                   12.0000                  0          
                  0 + 2.0000i              5.0000          
                  5.0000                    4.0000
```

### 矩阵的逆矩阵

1.对于方阵A，如果为非奇异方阵，则存在逆矩阵inv(A)

2.对于奇异矩阵或者非方阵，并不存在逆矩阵，但可以使用pinv(A)求其伪逆


















