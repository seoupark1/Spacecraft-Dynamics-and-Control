quiz6에서 sheppard's method를 사용할 때, largest value를 사용하는게 사실상 의미가 없다고 생각했다. 왜냐하면 b0(q0)가 0만 아니라면 singularity가 발생하지 않기에 그 부분만 확인하면 계산 결과는 동일할 것이라고 생각했기 때문이다.
하지만 coursera 강의 내에서 quiz 평가를 할 때 오류가 발생하는 것을 확인한 후, largest value를 추출하는 함수를 추가해 dcm을 euler parameters로 변환했더니 결과값의 소수점 굉장히 아래 부분에서 변화가 나타났고 오류가 사라졌다. 이를
통해서 소수점 몇십자리 아래의 숫자도 critical한 역할을 한다는 사실을 깨달을 수 있었다.


