[cmake](https://keunjun.blog/2019/03/05/cmake-%ed%94%84%eb%a1%9c%ec%a0%9d%ed%8a%b8-%ed%85%9c%ed%94%8c%eb%a6%bf/)

![image](https://user-images.githubusercontent.com/108650199/206074505-f5318252-0e08-4777-8c83-39f003dc37af.png)

### 6
- find_package 또는 include 함수에서 ./cmake/ 폴더 안에 있는 모듈들을 이용하기 위해서는 CMAKE_MODULE_PATH 에 cmake 폴더 주소를 넣어줘야 한다.
- set 함수를 이용해서 추가 할 수 있다.
### 8-9
- 예제를 빌드할 건지 말건지, API 문서를 생성할건지 말건지 등 사용자의 니즈에 맞춰서 cmake 를 수행할 수 있다.
- set 함수와 CACHE 옵션을 이용하면 사용자가 외부에서 cmake 의 변수를 조작할 수 있다.
- 예를 들어 BUILD_EXAMPLE 은 ON 으로, BUILD_DOC은 OFF로 하고 싶으면 ‘cmake -D BUILD_EXAMPLE=ON -D BUILD_DOC=OFF ..’와 같이 수행하면 된다. (참고: 변수와 캐시)
### 11-15
- find_package(SomePackage) 를 수행하면 ./cmake/FindSomePackage.cmake 가 실행되고 만약에 해당 라이브러리를 찾으면 SomePackage_FOUND 가 TRUE 가 된다.
- 만약에 찾지 못하는경우 14번째 줄로가서 에러 메세지를 출력한다.
### 19-21
- 만약에 cmake 옵션으로 BUILD_EXAMPLE 이 ON 또는 TRUE 세팅된 경우 20번째 줄이 수행된다.
- 아무 세팅도 하지 않는 경우 default 가 ON 이므로 결과는 똑같다.
- 20번째 줄은 example 디렉토리에 있는 CMakeLists.txt 를 실행시키라는 의미를 담고 있다.
### 22-30
- BUILD_DOC 이 ON 또는 TRUE 인 경우 API 문서를 만드는 함수를 실행한다.
- 정확하게 말하면 cmake 하는 도중에 만드는것이 아니고 add_custom_target 을 통해서 make doc 과 같이 사용자 정의 빌드 규칙을 만들어서 문서를 생성하게 된다.
- 현재 이 템플릿의 경우 make doc 을 하면 API 문서를 생성한다.
### 32
- 위에서 설명한것처럼 bundang-config.cmake.in 을 이용해서 bundang-config.cmake 을 만들어낸다.
- 자세한건 configure_file 을 참고하자.
### 33
- bundang-config.cmake 는 외부 CMake 프로젝트에서 현재 이 프로젝트에서 생성된 bundang 라이브러리를 사용하기 위해 필요한 파일이다.
- CMake 에는 이와 같은 config 파일에 경로를 정하는 규칙을 정해두었다.
- 규칙을 따르지 않는 경우 직접 경로를 설정해야하는 불편함이 있으므로 따르는 것이 좋다.
- 33번째 줄은 사용자가 make install 을 수행할때 bundang-config.cmake 파일을 적절한 위치에 복사 붙여넣기 하는 과정을 설정하고 있다.
