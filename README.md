# my_quadruped_project

<img width="1911" height="1174" alt="image" src="https://github.com/user-attachments/assets/e6e26970-382e-488c-bb53-fe120ee0f270" />


# URDF-to-MJCF Assembler for Quadruped Leg Modules

**Автор:** Шумкарбек кызы Нурзада, группа R4133c(507206)

## Цель проекта
Разработать программный модуль для автоматической сборки целостных моделей ног квадрупедного робота из отдельных URDF-модулей (бедро, голень, стопа, таз) с конвертацией в формат MuJoCo.

## Описание
Программный модуль для сборки целостных моделей квадрупедных роботов в MuJoCo из отдельных URDF-описаний модулей ноги.

## Функциональность
- Конвертация URDF моделей в формат MJCF (MuJoCo)
- Автоматическое объединение модулей ног с корпусом в единую модель
- Автоматическое создание иерархии тел и суставов
- Валидация структуры моделей

## Структура проекта
- quadruped-assembler
- src
- configs - модели роботов
- outputs  MJCF - собранные модили ног с торсом
- requirements.txt - ависимости
- LICENSE MIT лицензия
- README.md - документация

## Технологии
- Python 3.8+
- xml.etree.ElementTree
- MuJoCo (mjpro 2.3+)

## Установка
```bash
git clone https://github.com/Nurzada-sh/my_project_converter.git
cd my_project_converter
pip install -r requirements.txt
