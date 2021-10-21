#ifndef COLORMAPS_HPP
#define COLORMAPS_HPP

#include <iostream>


class Color {

    private:
        uint8_t r;
        uint8_t g;
        uint8_t b;
        double min,max;

    public:
        Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){}

        Color(double mn, double mx): min(mn), max(mx){}

        void setMinMax(double min, double max){ min = min; max = max;}

        void setMin(double min){min = min;}

        void setMax(double max){max = max;}

        // Simple color generator
        void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
            R = r;
            G = g;
            B = b;
        }

        void getColor(double &rd, double &gd, double &bd){
            rd = (double)r/255;
            gd = (double)g/255;
            bd = (double)b/255;
        }

        uint32_t getColor(){
            return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
        }

        // Complex color generator
        uint32_t getColor(double c){

            double normalized = (c - min)/(max-min) * 2 - 1;
            double rd, gd, bd;

            r = (int) (base(normalized - 0.5) * 255);
            g = (int) (base(normalized) * 255);
            b = (int) (base(normalized + 0.5) * 255);

            rd = (double)r/255;
            gd = (double)g/255;
            bd = (double)b/255;

            return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
        }

    private:
        double interpolate(double val, double y0, double x0, double y1, double x1){
            return (val - x0)*(y1-y0)/(x1-x0) + y0;
        }
        double base(double val){
            if (val <= -0.75) return 0;
            else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
            else if (val <= 0.25) return 1;
            else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
            else return 0;
        }
};


class ColorPalette {

    private:
        std::vector<Color> colors;

    public:

        ColorPalette() {}

        std::vector<Color> getColorPalette() {
            return colors;
        }

        void createColorPalette(int size){

            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;

            for (int i=0;i<size;i++){

                while (r<70 && g < 70 && b < 70){
                    r = rand()%(255);
                    g = rand()%(255);
                    b = rand()%(255);
                }

                Color c(r,g,b);
                r = 0;
                g = 0;
                b = 0;
                colors.push_back(c);
            }
        }

};

#endif
