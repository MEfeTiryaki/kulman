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
  echo -e ""
  echo -e "${COLOR_BOLD}KULLANIM${COLOR_RESET}"
  echo -e ""
  echo -e "${COLOR_BOLD}Açıklama${COLOR_RESET}"
  echo -e ""
  echo -e "${COLOR_BOLD}DİPÇELER${COLOR_RESET}"
  echo -e ""
  echo -e "${COLOR_BOLD}YAZARLAR${COLOR_RESET}"
  echo -e "       M.Efe Tiryaki"
  echo -e ""
  color_reset
}

# Set defaults
git_dir="~/git"
catkin_ws="~/catkin_ws"
repo_="hepsi"

# Değişlenleri oku
while getopts "g:c:s:h" opt; do
  case $opt in
    g) git_dir="$OPTARG"
    ;;
    c) catkin_ws="$OPTARG"
    ;;
    s) repo_="$OPTARG"
    ;;
    h) display_help
       exit
    ;;
    \?) echo "Geçersiz seçenek -$OPTARG" >&2
    ;;
  esac
done

# Şuanki konum
curr_path=pwd


if [ -d "`eval echo ${git_dir//>}`" ]; then
  printf "${COLOR_INFO}Git klasör %s bulundu.${COLOR_RESET}\n" "$git_dir"
  if [ -d "`eval echo ${catkin_ws//>}`" ]; then
    printf "${COLOR_INFO}Catkin çalışma ortamı %s bulundu.${COLOR_RESET}\n" "$catkin_ws"
    if [ "$repo_" = hepsi ] ; then
      printf "${COLOR_WARN}Bütün ilgili repolar indiriliyor${COLOR_RESET}\n"
      printf "${COLOR_INFO}%s saklantısı indiliyor.${COLOR_RESET}\n" "kulman"
      git clone https://github.com/Sadetra/kulman.git
      printf "${COLOR_INFO}%s saklantısı catkin çalışma ortamına bağlanıyor.${COLOR_RESET}\n" "kulman"
      ln -sr "$git_dir/kulman/" "$catkin_ws/src/"
      printf "${COLOR_INFO}%s saklantısı indiliyor.${COLOR_RESET}\n" "arac"
      git clone https://github.com/Sadetra/arac.git
      printf "${COLOR_INFO}%s saklantısı catkin çalışma ortamına bağlanıyor.${COLOR_RESET}\n" "arac"
      ln -sr "$git_dir/arac/" "$catkin_ws/src/"
    elif [ "$repo_" = kulman ]; then
      printf "${COLOR_INFO}%s saklantısı indiliyor.${COLOR_RESET}\n" "$repo_"
      git clone https://github.com/Sadetra/kulman.git
      printf "${COLOR_INFO}%s saklantısı catkin çalışma ortamına bağlanıyor.${COLOR_RESET}\n" "$repo_"
      ln -sr "$git_dir/$repo_/" "$catkin_ws/src/"
    elif [ "$repo_" = arac ]; then
      printf "${COLOR_INFO}%s saklantısı indiliyor.${COLOR_RESET}\n" "$repo_"
      git clone https://github.com/Sadetra/arac.git
      printf "${COLOR_INFO}%s saklantısı catkin çalışma ortamına bağlanıyor.${COLOR_RESET}\n" "$repo_"
      ln -sr "$git_dir/$repo_/" "$catkin_ws/src/"
    fi
  else
    printf "${COLOR_WARN}Catkin çalışma ortamı %s bulunamadı.\n${COLOR_BOLD}->EXIT${COLOR_RESET}\n" "$catkin_ws"
    exit
  fi
else
  printf "${COLOR_WARN}Git klasör %s bulunamadı.\n${COLOR_BOLD}->Çıkış${COLOR_RESET}\n" "$eclipse_ws"
  exit
fi
