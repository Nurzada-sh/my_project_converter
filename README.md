# my_quadruped_project

# URDF-to-MJCF Assembler for Quadruped Leg Modules

**Автор:** Шумкарбек кызы Нурзада, группа R4133c(507206)

## Цель проекта
Разработать программный модуль для автоматической сборки целостных моделей ног квадрупедного робота из отдельных URDF-модулей (бедро, голень, стопа, таз) с конвертацией в формат MuJoCo.

## Технологии
- Python 3.8+
- xml.etree.ElementTree
- MuJoCo (mjpro 2.3+)
- Docker

## Быстрый запуск
```bash
# Установка
pip install -r requirements.txt
