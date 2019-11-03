# 1、编程

## 1.1、 

打印出来lambda就行了



## 1.2、 

把“CurveFitting.cpp”中几行代码修改下就可以了。

<img src="/home/wang/workspace/VIO-SLAM/week3/图片1.png" alt="图片1" style="zoom:200%;" />

## 1.3





# 2、公式推导

$$
\begin{align}

f_{15} &= \frac{\partial \alpha_{b_i b_{k+1}} }{\partial \delta b^g_k}\\
&=\frac{\partial (\alpha_{b_i b_k} + \beta_{b_i b_k}\delta t+\frac{1}{2}a\delta t^2)}{\partial\delta b^g_k}\\
& 由于(\alpha_{b_i b_k} + \beta_{b_i b_k}\delta t)求导等于零 \\
&= \frac{\partial (\frac{1}{2}a\delta t^2)}{\partial\delta b^g_k} \\
&= \frac{\partial (\frac{1}{2}\frac{1}{2}(q_{b_i b_k}(a^{b_{k}}-b_k^a)+q_{b_i b_{k+1 }}(a^{b_{k+1}}-b_k^a))\delta t^2)}{\partial\delta b^g_k} \\
&= \frac{\partial (\frac{1}{2}\frac{1}{2}(q_{b_i b_{k+1 }}(a^{b_{k+1}}-b_k^a))\delta t^2)}{\partial\delta b^g_k} \\
&= \frac{1}{4}\frac{\partial q_{b_i b_k} \otimes \begin{bmatrix}{1} \\ {\frac{1}{2}\omega\delta t} \end{bmatrix} \otimes \begin{bmatrix}1 \\ \frac{1}{2}\delta b^g_k \delta t \end{bmatrix} (a^{b_{k+1}}-b_k^a)\delta t^2}{\partial\delta b^g_k} \\
&=\frac{1}{4}\frac{\partial R_{b_i b_{k+1}}(I+[-\delta b^g_k\delta t]_\times) (a^{b_{k+1}}-b_k^a)\delta t^2}{\partial\delta b^g_k}{\partial\delta b^g_k} \\
&= -\frac{1}{4} (R_{b_i b_{k+1}}[a^{b_{k+1}}-b_k^a)]_\times\delta t^2) (-\delta t)\\

\end{align}
$$




$$
\begin{align}

g_{12} &= \frac{\partial \alpha_{b_i b_{k+1}} }{\partial n^g_k}\\
&=\frac{\partial (\alpha_{b_i b_k} + \beta_{b_i b_k}\delta t+\frac{1}{2}a\delta t^2)}{\partial n^g_k}\\
&= \frac{\partial (\frac{1}{2}a\delta t^2)}{\partial\delta n^g_k} \\
&= \frac{\partial (\frac{1}{2}\frac{1}{2}(q_{b_i b_k}(a^{b_{k}}-b_k^a)+q_{b_i b_{k+1 }}(a^{b_{k+1}}-b_k^a))\delta t^2)}{\partial\delta n^g_k} \\
&= \frac{\partial (\frac{1}{2}\frac{1}{2}(q_{b_i b_{k+1 }}(a^{b_{k+1}}-b_k^a))\delta t^2)}{\partial\delta n^g_k} \\

&= \frac{1}{4}\frac{\partial q_{b_i b_k} \otimes \begin{bmatrix}{1} \\ {\frac{1}{2}\omega\delta t} \end{bmatrix} \otimes \begin{bmatrix}1 \\ \frac{1}{4}\delta n^g_k \delta t \end{bmatrix} (a^{b_{k+1}}-b_k^a)\delta t^2}{\partial\delta n^g_k} \\

&=\frac{1}{4}\frac{\partial R_{b_i b_{k+1}}(I+[\frac{1}{2} \delta n^g_k\delta t]_\times) (a^{b_{k+1}}-b_k^a)\delta t^2}{\partial\delta n^g_k}{\partial\delta n^g_k} \\
&= -\frac{1}{4} (R_{b_i b_{k+1}}[a^{b_{k+1}}-b_k^a)]_\times\delta t^2) (\frac{1}{2}\delta t)\\

\end{align}
$$

# 3、证明


$$
\because (J^TJ+\mu I)\triangle x_{lm}=-J^Tf \\
(V\land V^T+\mu I)\triangle x_{lm}=-J^Tf \\

\therefore  (V(\land +\mu I)V^T)\triangle x_{lm} =-J^T f\\

\because F^`(x)=(J^Tf)^T \\
\therefore V^T\triangle x_{lm}=-(\land +\mu I)^{-1}V^TF^`(x)^T \\

\therefore \triangle x_{lm}=- V(\land +\mu I)^{-1}V^TF^`(x)^T \\

\therefore 得证
$$
