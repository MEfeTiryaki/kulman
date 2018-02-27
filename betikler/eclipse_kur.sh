# Terminal rengi sıfırlar
function color_reset
{
  echo -ne "\033[0m"
}

# Kullanılan renkler
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_CODE="\033[0m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"

# yardım çıktıları
function display_help
{
  color_reset
  echo -e "${COLOR_BOLD}AD${COLOR_RESET}"
  echo -e "       eclipse_kur.sh - İşbu betik catkin çalışma ortamınızı eclipse içine kurar"
  echo -e ""
  echo -e "${COLOR_BOLD}KULLANIM${COLOR_RESET}"
  echo -e "       ${COLOR_BOLD}eclipse_kur.sh${COLOR_RESET} [${COLOR_UNDE}SEÇENEKLER${COLOR_RESET}]..."
  echo -e ""
  echo -e "       İşbu betiğin seçenekleri aşağıda dizilmiştir;"
  echo -e ""
  echo -e "       ${COLOR_BOLD}-e 'EclipseWs_adı'${COLOR_RESET} - eclipse çalışma ortamının bulundğu klasör ${COLOR_INFO}(default: ~/workspace)${COLOR_RESET}"
  echo -e "       ${COLOR_BOLD}-c 'CatkinWs_adı'${COLOR_RESET}  - catkin çalışma ortamının bulunduğu klasör ${COLOR_INFO}(default: ~/catkin_ws)${COLOR_RESET}"
  echo -e "       ${COLOR_BOLD}-r ${COLOR_RESET}              - eclipse çalışma klasörünüzdeki üstverileri siler ${COLOR_INFO}(default: false)${COLOR_RESET}"
  echo -e ""
  echo -e "       ${COLOR_BOLD}--help${COLOR_RESET}, ${COLOR_BOLD}-h${COLOR_RESET}       - Yardım iletisini gösterir."
  echo -e ""
  echo -e ""
  echo -e "${COLOR_BOLD}Açıklama${COLOR_RESET}"
  echo -e "       İşbu betik derleme anında oluşan .project dosyalarını eclipse"
  echo -e "       çalışma ortamına ekler.Aşağıdaki adımları izleyerek kullanın."
  echo -e ""
  echo -e "       ${COLOR_ITEM}1):${COLOR_RESET} Derlemede .project dosyalarını oluşturmak için, "
  echo -e "           projenizi aşağıdaki komut ile derleyin."
  echo -e "           ${COLOR_INFO}catkin build -G\"Eclipse CDT4 - Unix Makefiles\" -DCMAKE_CXX_COMPILER_ARG1=-std=c++11${COLOR_RESET}"
  echo -e ""
  echo -e "           Eğer '${COLOR_INFO}catkin config${COLOR_RESET}' komudunu çalıştırdığınızda,'${COLOR_INFO}Additional CMake Args${COLOR_RESET}' "
  echo -e "           dizesinde yukardakine benzer bir ifade varsa doğrudan '${COLOR_INFO}catkin build${COLOR_RESET}' da kullanabilirsiniz."
  echo -e "           Bu değişkeni olağan catkin derleme durumu yapmak için ,"
  echo -e "           '${COLOR_INFO}catkin config -DCMAKE_BUILD_TYPE=Release -G\"Eclipse CDT4 - Unix Makefiles\" -DCMAKE_CXX_COMPILER_ARG1=-std=c++11${COLOR_RESET}'"
  echo -e ""
  echo -e "       ${COLOR_ITEM}2):${COLOR_RESET} ${COLOR_BOLD}./eclipse_kur -e ~/workspace -c ~/catkin_ws -r${COLOR_BOLD} "
  echo -e ""
  echo -e "       ${COLOR_ITEM}3):${COLOR_RESET} Son bohçanın eklenmesi 5 sn den fazla sürerse (ctrl+c) ile kapatın."
  echo -e "           HAZIRSINIZ!!"
  echo -e ""
  echo -e "       ${COLOR_ITEM}4):${COLOR_RESET} Terminal'den eclipse'i çalıştırın."
  echo -e "           ${COLOR_INFO}bash -i -c \"eclipse\"${COLOR_RESET}"
  echo -e ""
  echo -e "       ${COLOR_ITEM}5):${COLOR_RESET} Eclipse açıldığında, soldaki 'Project Explorer'da bütün bohçalarınız"
  echo -e "           bellirmiş olmalı. Kodunuzu bohçanın içindeki [Source directory] içinde bulabilirsiniz."
  echo -e "           Daha duru bir 'Project Explorer' için aşağı bakan üçgen tuş içinden 'Select Working Set'i kurcalayın..."
  echo -e ""
  echo -e "       ${COLOR_ITEM}6):${COLOR_RESET} Kodlarınız doğru şekilde renklendirmek için, 'Project Explorer'dan bohça adına"
  echo -e "           sağ tıklayın. 'Index>Rebuild'e basın. Eclipse ekranın sağ altındaki %XXX dolunca kodunuz hazır. "
  echo -e "           Daha sonra 'Index>Update with modified files'ı kullanın. Artık kendi yazdığınız nesnelerde özbitirilecek."
  echo -e ""
  echo -e "${COLOR_BOLD}DİPÇELER${COLOR_RESET}"
  echo -e ""
  echo -e "       ${COLOR_ITEM}1):${COLOR_RESET} Aşağıdaki kısım .project'leri ekler."
  echo -e "           ${COLOR_INFO}-G\"Eclipse CDT4 - Unix Makefiles\"${COLOR_RESET}"
  echo -e ""
  echo -e "           Aşağıdaki kısım eclipse'in c++11 ayıkmasını sağlar."
  echo -e "           ${COLOR_INFO}-DCMAKE_CXX_COMPILER_ARG1=-std=c++11${COLOR_RESET}"
  echo -e ""
  echo -e "       ${COLOR_ITEM}2):${COLOR_RESET} Projeniz bir kısmı c++11 değil ise, "
  echo -e "           1. c++11 olmadan derleyin "
  echo -e "              e.g catkin build -G\"Eclipse CDT4 - Unix Makefiles\" <bohça_adı>"
  echo -e "           2. geri kalanı c++ ile derleyin "
  echo -e "              e.g catkin build -G\"Eclipse CDT4 - Unix Makefiles\" -DCMAKE_CXX_COMPILER_ARG1=-std=c++11"
  echo -e ""
  echo -e "       ${COLOR_ITEM}3):${COLOR_RESET} Bu işlemden sonra eclipse'i terminalde aşağıdaki komutla çağırmayı unutmayın.".
  echo -e "           ${COLOR_INFO}bash -i -c \"eclipse\"${COLOR_RESET}"
  echo -e ""
  echo -e "       ${COLOR_ITEM}4):${COLOR_RESET} Eclipse'de daha önceden yapmış olduğunuz arayüz değişiklikleri de sıfırlanacak.".
  echo -e "           Bu yüzden bu işlemi yapmadan önce 'File>Export>General>Preferences' ile arayüzünüzü kaydetin. "
  echo -e "           Sonrasında 'File>Import>General>Preferences' ile geri yükleyebilirsiniz."
  echo -e ""
  echo -e "${COLOR_BOLD}YAZARLAR${COLOR_RESET}"
  echo -e "       Gabriel Hottiger: öz yazar"
  echo -e "       M.Efe Tiryaki: Türkçeye uydurdu ve çeşitli açıklamalar ekledi."
  echo -e ""
  color_reset
}

# Set defaults
eclipse_ws="workspace"
catkin_ws="catkin_ws"
clear_eclipse=false

# Değişlenleri oku
while getopts "e:c:rh" opt; do
  case $opt in
    e) eclipse_ws="$OPTARG"
    ;;
    c) catkin_ws="$OPTARG"
    ;;
    r) clear_eclipse=true
    ;;
    h) display_help
       exit
    ;;
    \?) echo "Geçersiz seçenek -$OPTARG" >&2
    ;;
  esac
done

# Get current directory
curr_path=pwd

# Eclipse çalışma ortamını bul
if [ -d "`eval echo ${eclipse_ws//>}`" ]; then
  printf "${COLOR_INFO}Eclipse çalışma ortanı %s bulundu.${COLOR_RESET}\n" "$eclipse_ws"
  # Catkin çalışma ortamını bul
  if [ -d "`eval echo ${catkin_ws//>}`" ]; then
    printf "${COLOR_INFO}Catkin çalışma ortamı %s bulundu.${COLOR_RESET}\n" "$catkin_ws"

    # klasörler kaldırılıyor
    if [ "$clear_eclipse" = true ] ; then
      printf "${COLOR_WARN}rm %s/.metadata${COLOR_RESET}\n" "$eclipse_ws"
      eval rm -rf "$eclipse_ws/.metadata"
      printf "${COLOR_WARN}rm %s/RemoteSystemsTempFiles${COLOR_RESET}\n" "$eclipse_ws"
      eval rm -rf "$eclipse_ws/RemoteSystemsTempFiles"
    fi

    # .project dosyaları değiştiriliyor
    echo -n -e "${COLOR_ITEM}Modify .project files${COLOR_RESET}..."
    eval cd "$catkin_ws/build"
    # build içindeki her bohçayı bul
    for PROJECT in `find $PWD -name .project`; do
        DIR=`dirname $PROJECT`
        cd $DIR
        # Büyülü dize
        awk -f $(rospack find mk)/eclipse.awk .project > .project_with_env && mv .project_with_env .project
        echo -n "."
    done
    echo -e "Done"

    # import all projects in headless mode
    echo -e "${COLOR_BOLD}Bir bohça 5 sn uzun sürerse, (ctrl+c) ile kapatın.${COLOR_RESET}\n"
    sleep 3
    eclipse -nosplash -data "`eval echo ${eclipse_ws//>}`" -application org.eclipse.cdt.managedbuilder.core.headlessbuild -importAll "`eval echo ${catkin_ws//>}/build`" -D -std=c++11

  else
    printf "${COLOR_WARN}Catkin çalışma ortamı %s bulunamadı.\n${COLOR_BOLD}->EXIT${COLOR_RESET}\n" "$catkin_ws"
    exit
  fi
else
  printf "${COLOR_WARN}Eclipse çalışma ortamı %s bulunamadı.\n${COLOR_BOLD}->EXIT${COLOR_RESET}\n" "$eclipse_ws"
  exit
fi
